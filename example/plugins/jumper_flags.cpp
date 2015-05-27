/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <sstream>
#include <Pacer/utilities.h>
#include <Pacer/controller.h>

std::string plugin_namespace;

using namespace Pacer;
using namespace Ravelin;
boost::shared_ptr<Pacer::Controller> ctrl_ptr;

class JointPID {
public:
  int phase;

  JointPID(int p) { phase = p;  init();  }

  Ravelin::VectorNd u;

  struct Gains
  {
    double perr_sum;
    double kp;
    double kv;
    double ki;
  };

  std::vector<Gains> _gains;

  Ravelin::VectorNd q_des,
          qd_des,
          q,
          qd;

  void init() {

    std::map<std::string, std::vector<Gains> > gains;

    static std::vector<std::string>
    joint_names = ctrl_ptr->get_data<std::vector<std::string> >("init.joint.id");

    OUTLOG(phase, "PHASE", logERROR);

    OUTLOG(joint_names, "joint_names", logERROR);
    std::vector<double>
    Kp_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "liftoff_gains.kp"),
    Kv_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "liftoff_gains.kv"),
    Ki_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "liftoff_gains.ki");

    if (phase == 1) {
      OUTLOG(phase, "PHASE ONE", logERROR);
      Kp_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "landing_gains.kp");
      Kv_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "landing_gains.kv");
      Ki_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "landing_gains.ki");
    }

    std::vector<double>
    Kp = Kp_tmp,
    Kv = Kv_tmp,
    Ki = Ki_tmp,
    dofs = ctrl_ptr->get_data<std::vector<double> >("init.joint.dofs");

    OUTLOG(Kp, "Kp", logERROR);
    OUTLOG(Kv, "Kv", logERROR);
    OUTLOG(Ki, "Ki", logERROR);
    OUTLOG(dofs, "dofs", logERROR);

    int num_dofs = std::accumulate ( dofs.begin( ) , dofs.end( ) , 0.0 ) ;
    assert(num_dofs == Kp.size());
    assert(num_dofs == Kv.size());
    assert(num_dofs == Ki.size());

    for (int i = 0, ii = 0; i < joint_names.size(); i++) {
      gains[joint_names[i]] = std::vector<Gains>(dofs[i]);
      for (int j = 0; j < dofs[i]; j++, ii++) {
        gains[joint_names[i]][j].kp = Kp[ii];
        gains[joint_names[i]][j].kv = Kv[ii];
        gains[joint_names[i]][j].ki = Ki[ii];
        gains[joint_names[i]][j].perr_sum = 0;
      }
    }

    ctrl_ptr->convert_to_generalized<Gains>(gains, _gains);

    OUT_LOG(logERROR) << "Controller: " << plugin_namespace << " inited!";
  }

  void update() {
    qd_des.set_zero();

    Ravelin::VectorNd perr = q;
    perr -= q_des;
    Ravelin::VectorNd derr = qd;
    derr -= qd_des;

    assert(q.rows() == _gains.size());
    u.set_zero(q.rows());

    for (int i = 0; i < q.rows(); i++)
    {
      const double KP = _gains[i].kp;
      const double KV = _gains[i].kv;
      const double KI = _gains[i].ki;

      OUTLOG(KP, "NEW GAINS KP", logERROR);

      _gains[i].perr_sum += perr[i];
      double ierr = _gains[i].perr_sum;

      u[i] = -(perr[i] * KP + derr[i] * KV + ierr * KI);
    }
  }
};

void activate_joint_pid(boost::shared_ptr<Pacer::Controller> ctrl, JointPID pid) {
  pid.q_des  = ctrl->get_joint_generalized_value(Pacer::Controller::position_goal);
  pid.qd_des = ctrl->get_joint_generalized_value(Pacer::Controller::velocity_goal);
  pid.q  = ctrl->get_joint_generalized_value(Pacer::Controller::position);
  pid.qd = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);

  OUTLOG(pid.q, "joint_pid_q", logDEBUG);
  OUTLOG(pid.qd, "joint_pid_qd", logDEBUG);
  OUTLOG(pid.q_des, "joint_pid_q_des", logDEBUG);
  OUTLOG(pid.qd, "joint_pid_qd_des", logDEBUG);

  pid.update();

  Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
  OUTLOG(pid.u, "joint_pid_U", logDEBUG);
  u += pid.u;
  ctrl->set_joint_generalized_value(Pacer::Controller::load_goal, u);
}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t) {
  ctrl_ptr = ctrl;
  static JointPID pid_jumping(0);
  static JointPID pid_landing(1);

  int USE_DES_CONTACT = ctrl->get_data<int>(plugin_namespace + "des-contact");
  int HAS_JUMPED = ctrl->get_data<int>(plugin_namespace + "has_jumped");
  int HAS_STARTED = ctrl->get_data<int>(plugin_namespace + "has_started");
  int HAS_LANDED = ctrl->get_data<int>(plugin_namespace + "has_landed");

  std::vector<std::string>
  foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");

  std::vector<std::string> active_feet;

  int num_feet = foot_names.size();

  active_feet = foot_names;

  double activation_tol = 0;
  ctrl->get_data<double>(plugin_namespace + "min-allowed-friction", activation_tol);

  std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > contacts;
  for (int i = 0; i < active_feet.size(); i++) {
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(active_feet[i], c);
    if (!c.empty())
      if (c[0]->mu_coulomb >= activation_tol)
        contacts.push_back(c[0]);
  }


  int NC = contacts.size();

  if (NC > 0) {
    if (!HAS_STARTED)
      ctrl->set_data<int>(plugin_namespace + "has_started", 1);

    if (HAS_JUMPED && !HAS_LANDED)
      ctrl->set_data<int>(plugin_namespace + "has_landed", 1);

    /// Current State
    std::vector<double> x_des = ctrl->get_data<std::vector<double> >(plugin_namespace + "desired.x");
    std::vector<double> xd_des = ctrl->get_data<std::vector<double> >(plugin_namespace + "desired.xd");

    Ravelin::VectorNd
    generalized_qd = ctrl->get_generalized_value(Pacer::Controller::velocity),
    generalized_q  = ctrl->get_generalized_value(Pacer::Controller::position);

    boost::shared_ptr<Ravelin::Pose3d> base_frame( new Ravelin::Pose3d(
          ctrl->get_data<Ravelin::Pose3d>("base_stability_frame")));

    base_frame->update_relative_pose(Moby::GLOBAL);

    //  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Pose(*(base_frame.get()),0.1,1)));

    Ravelin::VectorNd center_of_mass_x = Utility::pose_to_vec(base_frame);
    Ravelin::Origin3d roll_pitch_yaw;
    Ravelin::Quatd(center_of_mass_x[3], center_of_mass_x[4], center_of_mass_x[5], center_of_mass_x[6]).to_rpy(roll_pitch_yaw[0], roll_pitch_yaw[1], roll_pitch_yaw[2]);

    int NDOFS = generalized_qd.rows();
    int NUM_JOINT_DOFS = NDOFS - NSPATIAL;

    Ravelin::VectorNd base_qd = generalized_qd.segment(NUM_JOINT_DOFS, NDOFS); // ctrl->get_base_value(Pacer::Controller::velocity);
    //Ravelin::Vector3d center_of_mass_x = ctrl->get_data<Ravelin::Vector3d>("center_of_mass.x");
    Ravelin::VectorNd vel_base(6), pos_base(6);
    for (int i = 0; i < 3; i++) {
      vel_base[i] = base_qd[i];
      vel_base[i + 3] = base_qd[3 + i];
      pos_base[i] = center_of_mass_x[i];
      pos_base[i + 3] = roll_pitch_yaw[i];
    }

    /// Jacobians
    Ravelin::MatrixNd N, S, T, D;

    ctrl->calc_contact_jacobians(generalized_q, contacts, N, S, T);

    D.set_zero(NDOFS, NC * 4);
    D.set_sub_mat(0, 0, S);
    D.set_sub_mat(0, NC, T);
    S.negate();
    T.negate();
    D.set_sub_mat(0, NC * 2, S);
    D.set_sub_mat(0, NC * 3, T);

    int nk = D.columns() / NC;
    int nvars = NC + NC * (nk);
    // setup R
    Ravelin::MatrixNd R(NDOFS, NC + (NC * nk) );
    R.block(0, NDOFS, 0, NC) = N;
    R.block(0, NDOFS, NC, NC * nk + NC) = D;

    Ravelin::SharedConstMatrixNd Jb = R.block(NUM_JOINT_DOFS, NDOFS, 0, NC * 3);
    Ravelin::SharedConstMatrixNd Jq = R.block(0, NUM_JOINT_DOFS, 0, NC * 3);

    double alpha = 1;

    //jump angle
    double g = 9.81;
    //staring angle
    double theta = 60;
    //double theta_radians = theta * M_PI / 180;

    //distance goal in meters
    double r = 1.0;
    //desired velocity in m/s
    double v_desired = std::sqrt((g * r) / (std::sin(2 * theta)));
    // forward value of desired vel
    double fwd_vel_des = v_desired * std::cos(theta);
    // upward value of desired vel
    double up_vel_des = v_desired * std::sin(theta);

    double upper_bound = 1;
    double fwd_pow = std::pow(t, 18);
    double up_pow = std::pow(t, 20);
    double theta_pow = std::pow(t, 5);

    //3 indices 0, .5, 1
    /*
    OUTLOG(T,"T",logDEBUG1);
      for(int cp=0;cp<control_points.size();cp++){
        OUTLOG(control_points[cp],"control_point",logDEBUG1);
      }
      OUTLOG(xd,"control_point_V",logDEBUG1);
      for(int d=0;d<3;d++){
        VectorNd           X(n);
        VectorNd          &coefs = spline_coef[i][d];
        
        //point spline needs  to be at at each point in time
        //T[i] is the point in time that X[i] coincides with
        for(int j=0;j<n;j++)
          X[j] = control_points[j][d];
        OUTLOG(X,"X",logDEBUG1);
        
        //Ravelin::Vector2d(xd[d],xd[d]) == beginning vel and end vel (not magnitude) in dimension d 
        //fwd_vel_des = xd[0]
        //up_vel_des = xd[2]
        //coefs static!! don't recalculate
        //if statement if coefs empty then calculate, otherwise skip
        Utility::calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(xd[d],xd[d]),coefs);

        // then re-evaluate spline
        // NOTE: this will only work if we replanned for a t_0  <  t  <  t_0 + t_I
        OUT_LOG(logDEBUG) << "Eval first step in spline";
        Utility::eval_cubic_spline(coefs,T,t,x[d],xd[d],xdd[d]);
      }
      }
     */

//set your desired velocity vector
    Ravelin::VectorNd Vb_des(6);
    if (!HAS_JUMPED) {
      Vb_des[0] = (fwd_pow < upper_bound ) ? fwd_pow : fwd_vel_des;
      Vb_des[1] = 0.0;
      Vb_des[2] = (up_pow < upper_bound ) ? up_pow : up_vel_des ;
      Vb_des[3] = 0.0;
      Vb_des[4] = (theta_pow < upper_bound ) ? theta_pow : -theta * M_PI / 180;
      Vb_des[5] = 0.0;


      OUTLOG(fwd_vel_des, "fwd_vel_des: ", logERROR);
      OUTLOG(vel_base[0], "vel_base[0]", logERROR);

      static std::vector<double>
      Kp = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.kp"),
      Kv = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.kv"),
      Ki = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.ki");

      Ravelin::VectorNd vel_base_dot(6);
      for (int j = 0; j < Vb_des.size(); j++) {
        vel_base_dot[j] = (Vb_des[j] - vel_base[j]) * Kv[j];
      }

      OUTLOG(Jb, "Jb: ", logERROR);
      OUTLOG(vel_base_dot, "vel_base", logERROR);

      //multiply Jb * desired base velocity to get desired foot velocity
      Ravelin::VectorNd Vft;
      Jb.transpose_mult(vel_base_dot  , Vft);

      //multiply desired foot velocity by alpha to get desired foot force
      Ravelin::VectorNd Fft = Vft;
      Fft *= -alpha;

      //Jq transpose * desired foot force to get desired torque
      Ravelin::VectorNd tau;
      Jq.mult(Fft, tau);

      /// Feedback gains
      OUTLOG(tau, "viip_fb", logERROR);

      Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
      u += tau;


      ctrl->set_joint_generalized_value(Pacer::Controller::load_goal, u);
    } else {
      OUTLOG(NC, "LANDING HERE", logERROR);
      activate_joint_pid(ctrl, pid_landing);
    }

  } else if (HAS_STARTED) {
    ctrl->set_data<int>(plugin_namespace + "has_jumped", 1);
    activate_joint_pid(ctrl, pid_jumping);
  }

}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"

