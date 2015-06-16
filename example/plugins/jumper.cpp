/****************************************************************************
 * Copyright 2015 Matt Scaperoth, Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <sstream>
#include <Pacer/utilities.h>
#include <Pacer/controller.h>
#include <Pacer/jumper.h>

std::string plugin_namespace;

using namespace Pacer;
using namespace Ravelin;
boost::shared_ptr<Pacer::Controller> ctrl_ptr;
boost::shared_ptr<Pose3d> base_link_frame;


void run_test(std::vector<Ravelin::VectorNd> coefs, VectorNd T_i, Vector3d real_x, Vector3d real_xd, Vector3d real_xdd, Vector3d xd_final, int n) {

  for (double t_step = T_i[0]; t_step <= T_i[n - 1]; t_step += .001) {
    for (int dim = 0; dim < 3; dim++) {
      Utility::eval_cubic_spline(coefs[dim], T_i, t_step, real_x[dim], real_xd[dim], real_xdd[dim]);
    }

    OUTLOG(xd_final, "jumper_xd_final", logERROR);
    OUTLOG(real_x, "jumper_x", logERROR);
    OUTLOG(real_xd, "jumper_xd", logERROR);
    OUTLOG(real_xdd, "jumper_xdd", logERROR);

  }

}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t) {
  ctrl_ptr = ctrl;


  int PHASE = (int)ctrl->get_data<double>(plugin_namespace + "phase");
  double theta = ctrl->get_data<double>(plugin_namespace + "des_jump_angle");
  double alpha = ctrl->get_data<double>(plugin_namespace + "alpha");;
  double g = ctrl->get_data<double>(plugin_namespace + "gravity");
//distance goal in meters
  double r = ctrl->get_data<double>(plugin_namespace + "range");

  //boolean for whether or not we're in development mode
  bool development = ctrl->get_data<bool>(plugin_namespace + "development");

  //variable on whether or not to kill program at end of spline
  bool time_constraint = ctrl->get_data<bool>(plugin_namespace + "time_constraint");

  double total_spline_time = ctrl->get_data<double>(plugin_namespace + "total_spline_time");

  std::vector<double>
  start_spline = ctrl->get_data<std::vector<double> >(plugin_namespace + "start_spline"),
  end_spline = ctrl->get_data<std::vector<double> >(plugin_namespace + "end_spline");

  OUTLOG(theta, "jumper_traj_theta", logERROR);
  OUTLOG(alpha, "jumper_traj_alpha", logERROR);
  OUTLOG(g, "jumper_traj_g", logERROR);
  OUTLOG(r, "jumper_traj_r", logERROR);


  //bound for simple polynomial spline to stop calculating
  //Ravelin::VectorNd exit_velocity(6);
  //Ravelin::VectorNd exit_velocity_dot(6);

  std::vector<std::string>
  foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");

  std::vector<std::string> active_feet;

  int NUM_FEET = foot_names.size();
  static std::vector< std::vector<VectorNd> > spline_coef(NUM_FEET);

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

  ///////////////////////////////////////////////
  ///
  /// CURRENT STATE
  ///
  ///////////////////////////////////////////////
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

  //the current values
  Vector3d
  real_x(pos_base[0], pos_base[1], pos_base[2]),
         real_xd(vel_base[0], vel_base[1], vel_base[2]),
         real_xdd(vel_base[3], vel_base[4], vel_base[5]);

  //the values to be manipulated by the spline
  Vector3d x = real_x,
           xd = real_xd,
           xdd = real_xdd;

  OUTLOG(x, "jumper_real_x", logERROR);
  OUTLOG(xd, "jumper_real_xd", logERROR);
  OUTLOG(xdd, "jumper_real_xdd", logERROR);

  ///////////////////////////////////////////////
  ///
  /// IF CONTACTS
  ///
  ///////////////////////////////////////////////
  if (NC > 0 ) {
    if (PHASE ==  LOADED && NC == 4)
      ctrl->set_data<double>(plugin_namespace + "phase", STARTED);

    if (PHASE ==  LIFTOFF)
      ctrl->set_data<double>(plugin_namespace + "phase", LANDED);



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


    ///////////////////////////////////////////////
    ///
    ///   TRAJECTORY CODE
    ///
    ///////////////////////////////////////////////

    double theta_radians = theta * M_PI / 180;
    double v_desired = std::sqrt((g * r) / (std::sin(2 * theta_radians)));
    //OUTLOG(v_desired, "jumper_v_desired", logERROR);
    // forward value of desired vel
    double fwd_vel_des = v_desired * std::cos(theta_radians);
    OUTLOG(fwd_vel_des, "jumper_fwd_vel_des", logERROR);
    // upward value of desired vel
    double up_vel_des = v_desired * std::sin(theta_radians);
    OUTLOG(up_vel_des, "jumper_up_vel_des", logERROR);



    ////////////////////////////////////////////////////////////
    //
    //  Code for jumper eef PID controller
    //
    ///////////////////////////////////////////////////////////
    std::vector<Vector3d>
    foot_vel(NUM_FEET),
             foot_pos(NUM_FEET),
             foot_acc(NUM_FEET),
             foot_init(NUM_FEET),
             foot_init_xd(NUM_FEET);
    std::vector<std::string> eef_names = ctrl_ptr->get_data<std::vector<std::string> >("jumper_eef-PID-controller.id");
    if (PHASE == STARTED) {
      for (int i = 0; i < NUM_FEET; i++) {
        //get initail position
        foot_init[i] = ctrl->get_data<Vector3d>(foot_names[i] + ".init.x");

        //populate goal foot position
        foot_pos[i] = foot_init[i];
        ctrl->get_data<Vector3d>(foot_names[i] + ".goal.x", foot_pos[i]);

        //get initial velocity
        foot_init_xd[i] = ctrl->get_data<Vector3d>(foot_names[i] + ".init.xd");

        //populate goal foot velocity
        foot_vel[i] = Vector3d(0, 0, 0, base_frame);
        ctrl->get_data<Vector3d>(foot_names[i] + ".goal.xd", foot_vel[i]);

        ctrl->set_data<Ravelin::Vector3d>(eef_names[i] + ".state.x", foot_pos[i]);
        ctrl->set_data<Ravelin::Vector3d>(eef_names[i] + ".state.xd", foot_vel[i]);

        ctrl->set_data<Ravelin::Vector3d>(eef_names[i] + ".goal.x", foot_init[i]);
        ctrl->set_data<Ravelin::Vector3d>(eef_names[i] + ".goal.xd", foot_init_xd[i]);

      }
    }


    ///////////////////////////////////////////////
    ///
    ///   SPLINE CODE
    ///
    ///////////////////////////////////////////////

    static Vector3d init_pos = x;
    Vector3d xd_final(fwd_vel_des, 0.0, up_vel_des);
    //each

    //T for spline code
    VectorNd T_i(2);
    T_i[0] = 0.0;
    T_i[1] = total_spline_time;
    //T_i[2] = 3.0;
    //T_i[3] = 3.5;

    //size of spline time vector
    int n = T_i.size();

    Ravelin::Vector3d
    init_spline_pos(start_spline[0], start_spline[1], start_spline[2]),
                    //needs to be changed to something meaningful for "squat" position
                    //lowest_spline_pos(.1, 0, -.02 ),
                    //rise_spline_pos(.9, 0, -.1 ),
                    final_spline_pos(end_spline[0], end_spline[1], end_spline[2]);

    ////////////////////////////////////////////////////////////
    //
    //  variable to turn testing and graph creation on or off
    //  run source parse_data.sh after activating development
    //
    ///////////////////////////////////////////////////////////
    static std::vector<VectorNd> coefs(4);

    for (int d = 0; d < 3; d++) {

      //Ravelin::Vector2d(xd[d],xd[d]) == beginning vel and end vel (not magnitude) in dimension d
      //fwd_vel_des = xd[0]
      //up_vel_des = xd[2]
      //coefs static!! don't recalculate
      //if statement if coefs empty then calculate, otherwise skip
      if (coefs[d].size() == 0) {

        VectorNd           X(n);

        //OUTLOG(coefs, "coefs here", logERROR);
        //point spline needs  to be at at each point in time
        //T[i] is the point in time that X[i] coincides with
        //
        // X =                i
        //           0           1           2
        //    0     x0      x_lowpoint    x_final_pos
        // d  1     y0      y_lowpoint    y_final_pos
        //    2     z0      z_lowpoint    z_final_pos
        //
        // for the x,y,z_final_pos find the outermost point of the movement by growing the vector ("exit" position)
        X[0] = init_spline_pos[d];
        X[1] = final_spline_pos[d];
        //X[2] = rise_spline_pos[d];
        //X[3] = final_spline_pos[d];

        OUTLOG(X, "jumper_spline_X", logERROR);

        Utility::calc_cubic_spline_coefs(T_i, X, Ravelin::Vector2d(xd[d], xd_final[d]), coefs[d]);
      }

      // then evaluate spline
      OUT_LOG(logDEBUG) << "Eval first step in spline";

      Utility::eval_cubic_spline(coefs[d], T_i, t, x[d], xd[d], xdd[d]);

    }


    if (development) {
      run_test( coefs,  T_i,  real_x, real_xd, real_xdd, xd_final, n);
      ctrl->set_data<bool>(plugin_namespace + "development", false);
    }

    if (PHASE < LANDED) {
      Ravelin::VectorNd xb_des(6);
      //set your desired velocity vector
      xb_des[0] = x[0];
      xb_des[1] = x[1];
      xb_des[2] = x[2];
      xb_des[3] = 0.0;
      xb_des[4] = 0.0;
      xb_des[5] = 0.0;

      Ravelin::VectorNd xdb_des(6);
      //set your desired velocity vector
      xdb_des[0] = xd[0];
      xdb_des[1] = xd[1];
      xdb_des[2] = xd[2];
      xdb_des[3] = 0.0;
      xdb_des[4] = 0.0;
      xdb_des[5] = 0.0;

      Ravelin::VectorNd xddb_des(6);
      //set your desired velocity vector
      xddb_des[0] = xdd[0];
      xddb_des[1] = xdd[1];
      xddb_des[2] = xdd[2];
      xddb_des[3] = 0.0;
      xddb_des[4] = 0.0;
      xddb_des[5] = 0.0;

      OUTLOG(vel_base, "jumper_vel_base", logERROR);

      static std::vector<double>
      Kp = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.kp"),
      Kv = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.kv"),
      Ki = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.ki");

      Ravelin::VectorNd vel_base_dot(6);
      for (int j = 0; j < xdb_des.size(); j++) {
        //exit_velocity[j] = vel_base[j];
        //exit_velocity_dot[j] = (Vb_des[j] - vel_base[j]) * Kv[j];
        vel_base_dot[j] = (xdb_des[j] - vel_base[j]) * Kv[j];
      }

      OUTLOG(xdb_des, "jumper_vb_des: ", logERROR);
      OUTLOG(vel_base_dot, "vel_base_dot", logERROR);
      OUTLOG(Jb, "Jb", logERROR);

      //multiply Jb * desired base velocity to get desired foot velocity
      Ravelin::VectorNd Vft;
      Jb.transpose_mult(vel_base_dot  , Vft);

      OUTLOG(Vft, "Vft", logERROR);

      //multiply desired foot velocity by alpha to get desired foot force
      Ravelin::VectorNd Fft = Vft;
      Fft *= -alpha;

      OUTLOG(Fft, "Fft", logERROR);

      OUTLOG(Jq, "Jq", logERROR);

      //Jq transpose * desired foot force to get desired torque
      Ravelin::VectorNd tau;
      Jq.mult(Fft, tau);


      OUTLOG(Fft, "Fft", logERROR);

      /// Feedback gains
      OUTLOG(tau, "viip_fb", logERROR);

      Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
      u += tau;

      ctrl->set_joint_generalized_value(Pacer::Controller::load_goal, u);
    } else {
      ctrl->set_data<double>(plugin_namespace + "phase", LANDED);

    }

  } else if (PHASE == STARTED || PHASE == LANDED) {
    OUTLOG(PHASE, "LIFTOFF HERE", logERROR);

    if (PHASE == STARTED) {
      //OUTLOG(exit_velocity, "exit_velocity", logERROR);
      //OUTLOG(exit_velocity_dot, "exit_velocity_dot", logERROR);
    }

    ctrl->set_data<double>(plugin_namespace + "phase", LIFTOFF);


  } else if (PHASE == LIFTOFF) {

  }

  if (time_constraint && t >= total_spline_time) {
    exit(1);
  }
  OUTLOG(t, "jumper_time", logERROR);

}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"

