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

int LOADED = 0;
int STARTED = 1;
int LIFTOFF = 2;
int LANDED = 3;

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

    OUTLOG(joint_names, "joint_names", logERROR);


    std::vector<double>
    Kp_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "liftoff_gains.kp"),
    Kv_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "liftoff_gains.kv"),
    Ki_tmp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace + "liftoff_gains.ki");

    if (phase == LANDED) {
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
  static JointPID pid_liftoff(LIFTOFF);
  static JointPID pid_land(LANDED);

  int PHASE = (int)ctrl->get_data<double>(plugin_namespace + "phase");
  double theta = ctrl->get_data<double>(plugin_namespace + "des_jump_angle");
  double alpha = ctrl->get_data<double>(plugin_namespace + "alpha");;
  double g = ctrl->get_data<double>(plugin_namespace + "gravity");
//distance goal in meters
  double r = ctrl->get_data<double>(plugin_namespace + "range");

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

  if (NC > 0) {
    if (PHASE ==  LOADED)
      ctrl->set_data<double>(plugin_namespace + "phase", STARTED);

    if (PHASE ==  LIFTOFF)
      ctrl->set_data<double>(plugin_namespace + "phase", LANDED);

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

    //desired velocity in m/s

    double theta_radians = theta * M_PI / 180;
    double v_desired = std::sqrt((g * r) / (std::sin(2 * theta_radians)));
    //OUTLOG(v_desired, "jumper_v_desired", logERROR);
    // forward value of desired vel
    double fwd_vel_des = v_desired * std::cos(theta_radians);
    OUTLOG(fwd_vel_des, "jumper_fwd_vel_des", logERROR);
    // upward value of desired vel
    double up_vel_des = v_desired * std::sin(theta_radians);
    OUTLOG(up_vel_des, "jumper_up_vel_des", logERROR);

    if (PHASE < LANDED) {

      ///////////////////////////////////////////////
      ///
      ///   SPLINE CODE
      ///
      ///////////////////////////////////////////////
      Vector3d x(pos_base[0], pos_base[1], pos_base[2]), xd(vel_base[0], vel_base[1], vel_base[2]), xdd(vel_base[3], vel_base[4], vel_base[5]);

      static Vector3d init_pos = x;

      OUTLOG(x, "jumper_pos_base", logERROR);
      std::vector<Vector3d>
      foot_vel(NUM_FEET),
               foot_pos(NUM_FEET),
               foot_acc(NUM_FEET),
               foot_init(NUM_FEET);

      //T for spline code
      VectorNd T_i(3);
      T_i[0] = 0.0;
      T_i[1] = 0.5;
      T_i[2] = 1.0;


      //each
      Ravelin::Vector3d
      init_spline_pos = init_pos,
      //needs to be changed to something meaningful for "squat" position
      lowest_spline_pos(0, 0, 0),
      final_spline_pos(fwd_vel_des * T_i[2], 0.0, up_vel_des * T_i[2] - (1 / 2 * g * pow(T_i[2], 2)));

      Vector3d xd_final(fwd_vel_des, 0.0, up_vel_des);
      // create spline using set of control points, place at back of history
      int n = T_i.size();

      static std::vector<VectorNd> coefs(3);

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
          X[1] = lowest_spline_pos[d];
          X[2] = final_spline_pos[d];

          OUTLOG(X, "jumper_spline_X", logERROR);

          Utility::calc_cubic_spline_coefs(T_i, X, Ravelin::Vector2d(xd[d], xd_final[d]), coefs[d]);
        }

        // then evaluate spline
        OUT_LOG(logDEBUG) << "Eval first step in spline";
        Utility::eval_cubic_spline(coefs[d], T_i, t, x[d], xd[d], xdd[d]);

      }

      OUTLOG(xd_final, "jumper_xd_final", logERROR);
      OUTLOG(x, "jumper_x", logERROR);
      OUTLOG(xd, "jumper_xd", logERROR);
      OUTLOG(xdd, "jumper_xdd", logERROR);

      Ravelin::VectorNd Vb_des(6);
      //set your desired velocity vector
      Vb_des[0] = xd[0];
      Vb_des[1] = xd[1];
      Vb_des[2] = xd[2];
      Vb_des[3] = 0.0;
      Vb_des[4] = 0.0;
      Vb_des[5] = 0.0;

      OUTLOG(vel_base, "jumper_vel_base", logERROR);

      static std::vector<double>
      Kp = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.kp"),
      Kv = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.kv"),
      Ki = ctrl->get_data<std::vector<double> >(plugin_namespace + "base_gains.ki");

      Ravelin::VectorNd vel_base_dot(6);
      for (int j = 0; j < Vb_des.size(); j++) {
        //exit_velocity[j] = vel_base[j];
        //exit_velocity_dot[j] = (Vb_des[j] - vel_base[j]) * Kv[j];
        vel_base_dot[j] = (Vb_des[j] - vel_base[j]) * Kv[j];
      }

      OUTLOG(Vb_des, "jumper_vb_des: ", logERROR);
      OUTLOG(vel_base_dot, "vel_base_dot", logERROR);

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
      ctrl->set_data<double>(plugin_namespace + "phase", LANDED);
      activate_joint_pid(ctrl, pid_land);
    }

  } else if (PHASE == STARTED || PHASE == LANDED) {
    OUTLOG(PHASE, "LIFTOFF HERE", logERROR);

    if (PHASE == STARTED) {
      //OUTLOG(exit_velocity, "exit_velocity", logERROR);
      //OUTLOG(exit_velocity_dot, "exit_velocity_dot", logERROR);
    }

    ctrl->set_data<double>(plugin_namespace + "phase", LIFTOFF);
    activate_joint_pid(ctrl, pid_liftoff);
  } else if (PHASE == LIFTOFF) {
    activate_joint_pid(ctrl, pid_liftoff);
  }

}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"

