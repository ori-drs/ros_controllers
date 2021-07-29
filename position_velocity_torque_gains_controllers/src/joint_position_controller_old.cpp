/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.*
 *  Copyright (c) 2021, Mathieu Geisert, Oxford Robotics Institute.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <position_velocity_torque_gains_controllers/joint_position_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace position_velocity_torque_gains_controllers
{

  JointPositionController::JointPositionController() {}
  ~JointPositionController::JointPositionController() {sub_command_.shutdown();}

  bool JointPositionController::init(hardware_interface::PositionVelocityTorqueGainsJointInterface* hw, ros::NodeHandle &n)
  {
    std::string joint_name;
    if (!n.getParam("joint", joint_name))
    {
      ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }
    joint_ = hw->getHandle(joint_name);
    
    // Load PID Controller using gains set on parameter server
    if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
      return false;

    double p, i, d, i_max, i_min;
    bool antiwindup;
    pid_controller_.getGains(p, i , d, i_min, i_max, antiwindup);
    joint_.setCommandVelocity(0.);
    joint_.setCommandEffort(0.);
    joint_.setCommandKp(p);
    joint_.setCommandKd(d);

    sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::commandCB, this);
    //ros_server_service_reload_gains_ = n..advertiseService("reload_gains", &JointPositionController::reloadGains, this);

    return true;
  }

  void JointPositionController::reloadGains(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) 
  {
    // Load PID Controller using gains set on parameter server
    pid_controller_.init(ros::NodeHandle(n, "pid")))

    double p, i, d, i_max, i_min;
    bool antiwindup;
    pid_controller_.getGains(p, i , d, i_min, i_max, antiwindup);
    joint_.setCommandVelocity(0.);
    joint_.setCommandEffort(0.);
    joint_.setCommandKp(p);
    joint_.setCommandKd(d);
  }

  void JointPositionController::starting(const ros::Time& time) 
  {
    double pos_command = joint_.getPosition();
    joint_.setCommandPosition(pos_command);
  }

  void JointPositionController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) 
  {
    joint_.setCommandPosition(*command_buffer_.readFromRT());
  }

  void JointPositionToJPVTPIDController::commandCB(const std_msgs::Float64ConstPtr& msg) 
  {
    command_buffer_.writeFromNonRT(msg->data);
  }
} //end namespace position_velocity_torque_gains_controllers

PLUGINLIB_EXPORT_CLASS(position_velocity_torque_gains_controllers::JointPositionController, controller_interface::ControllerBase)
