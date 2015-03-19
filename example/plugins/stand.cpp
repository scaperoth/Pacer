/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
      Ravelin::VectorNd 
        q_goal = ctrl->get_data<Ravelin::VectorNd>("init.q");
      Ravelin::VectorNd
        qd_goal = Ravelin::VectorNd::zero(q_goal.rows()),
        qdd_goal = Ravelin::VectorNd::zero(q_goal.rows());
      ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
      ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
      ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register_plugin"
