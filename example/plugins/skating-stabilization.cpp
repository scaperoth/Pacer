/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/utilities.h>
#include <Pacer/controller.h>
std::string plugin_namespace;

using namespace Pacer;
using namespace Ravelin;

void calc_base_correction(const std::vector<double>& Kp,const std::vector<double>& Kv,const std::vector<double>& Ki,const std::vector<double>& pos,const std::vector<double>& pos_des,const std::vector<double>& vel,const std::vector<double>& vel_des, Ravelin::VectorNd& base_correct){
  base_correct = Ravelin::VectorNd(6);

  OUTLOG(vel,"vel_base",logDEBUG1);
  OUTLOG(vel_des,"vel_des",logDEBUG1);
  OUTLOG(pos,"pos_base",logDEBUG1);
  OUTLOG(pos_des,"pos_des",logDEBUG1);

  static Ravelin::VectorNd sum_p_err = Ravelin::VectorNd::zero(6);
  for(int i=0;i<6;i++){
    sum_p_err[i]   += (pos_des[i] - pos[i]);
    base_correct[i] =   (vel_des[i] - vel[i])*Kv[i]
                      + (pos_des[i] - pos[i])*Kp[i]
                      + sum_p_err[i]*Ki[i];
  }

  OUTLOG(base_correct,"base_correct",logDEBUG);
}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
   std::vector<std::string>
        foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
        
  std::vector<std::string> active_feet;

  int num_feet = foot_names.size();
  int USE_DES_CONTACT = ctrl->get_data<int>(plugin_namespace+"des-contact");

  if(USE_DES_CONTACT){
    for(int i=0;i<foot_names.size();i++){
      int is_stance = 0;
      if(ctrl->get_data<int>(foot_names[i]+".stance",is_stance))
        if(is_stance == 1)
          active_feet.push_back(foot_names[i]);
    }
  } else
    active_feet = foot_names;

  double activation_tol = 0;
  ctrl->get_data<double>(plugin_namespace+"max-allowed-friction",activation_tol);
  
  std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > contacts;
  for(int i=0;i<active_feet.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(active_feet[i],c);
    if(!c.empty())
      if(c[0]->mu_coulomb < activation_tol)
        contacts.push_back(c[0]);
  }

  int NC = contacts.size();
  
  if(NC > 0){
    /// Current state
    std::vector<double> x_des = ctrl->get_data<std::vector<double> >(plugin_namespace+"desired.x");
    std::vector<double> xd_des = ctrl->get_data<std::vector<double> >(plugin_namespace+"desired.xd");
    
    Ravelin::VectorNd
    generalized_qd = ctrl->get_generalized_value(Pacer::Controller::velocity),
    generalized_q  = ctrl->get_generalized_value(Pacer::Controller::position);
    
    boost::shared_ptr<Ravelin::Pose3d> base_frame( new Ravelin::Pose3d(
                                                                       ctrl->get_data<Ravelin::Pose3d>("base_stability_frame")));
    
    base_frame->update_relative_pose(Moby::GLOBAL);
    
    Ravelin::VectorNd center_of_mass_x = Utility::pose_to_vec(base_frame);
    Ravelin::Origin3d roll_pitch_yaw;
    Ravelin::Quatd(center_of_mass_x[3],center_of_mass_x[4],center_of_mass_x[5],center_of_mass_x[6]).to_rpy(roll_pitch_yaw[0],roll_pitch_yaw[1],roll_pitch_yaw[2]);
    
    int NDOFS = generalized_qd.rows();
    int NUM_JOINT_DOFS = NDOFS - NSPATIAL;
    
    Ravelin::VectorNd base_qd = generalized_qd.segment(NUM_JOINT_DOFS,NDOFS);
    
    std::vector<double> vel_base(6), pos_base(6);
    for(int i=0;i<3;i++){
      vel_base[i] = base_qd[i];
      vel_base[i+3] = base_qd[3+i];
      pos_base[i] = center_of_mass_x[i];
      pos_base[i+3] = roll_pitch_yaw[i];
    }
    
    /// Contact Jacobians
    Ravelin::MatrixNd N,S,T,D;
    
    ctrl->calc_contact_jacobians(generalized_q,contacts,N,S,T);
    
    D.set_zero(NDOFS,NC*4);
    D.set_sub_mat(0,0,S);
    D.set_sub_mat(0,NC,T);
    S.negate();
    T.negate();
    D.set_sub_mat(0,NC*2,S);
    D.set_sub_mat(0,NC*3,T);
    
    int nk = D.columns()/NC;
    int nvars = NC + NC*(nk);
    // setup R
    Ravelin::MatrixNd R(NDOFS, NC + (NC*nk) );
    R.block(0,NDOFS,0,NC) = N;
    R.block(0,NDOFS,NC,NC*nk+NC) = D;
    
    /// Feedback Gains
    static std::vector<double>
      Kp = ctrl->get_data<std::vector<double> >(plugin_namespace+"gains.kp"),
      Kv = ctrl->get_data<std::vector<double> >(plugin_namespace+"gains.kv"),
      Ki = ctrl->get_data<std::vector<double> >(plugin_namespace+"gains.ki");

    /// Get desired base error-feedback force
    Ravelin::VectorNd base_correct;
    calc_base_correction(Kp,Kv,Ki,pos_base,x_des,vel_base,xd_des,base_correct);

//    {
//      Ravelin::SharedConstMatrixNd Jb = R.block(NUM_JOINT_DOFS,NDOFS,0,NC*3);
//      Ravelin::SharedConstMatrixNd Jq = R.block(0,NUM_JOINT_DOFS,0,NC*3);
//
//      Ravelin::VectorNd ws_correct;
//      Jb.transpose_mult(base_correct,ws_correct);
//      OUTLOG(ws_correct,"ws_correct",logDEBUG);
//      
//      // Remove non-compressive elements (cN < 0)
//      for(int i=0;i<NC;i++)
//        if(ws_correct[i] < 0.0)
//          ws_correct[i] = 0.0;
//      OUTLOG(ws_correct,"ws_correct (compressive)",logDEBUG);
//      
//      //   Remove Tangential Elements (for now)
//      //  for(int i=NC;i<ws_correct.rows();i++)
//      //      ws_correct[i] = 0.0;
//      //  OUTLOG(ws_correct,"ws_correct (normal)",logDEBUG);
//      
//      Jq.mult(ws_correct,js_correct,-1.0,0);
//      OUTLOG(js_correct,"js_correct",logDEBUG);
//    }
    
    
    
//    Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
//    u += fb;
//    ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u);
  }
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"

