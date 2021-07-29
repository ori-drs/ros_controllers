/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
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

#include <position_velocity_torque_gains_controllers/joint_effort_controller.h>
#include <pluginlib/class_list_macros.hpp>


namespace position_velocity_torque_gains_controllers {

JointEffortController::JointEffortController()
: command_(0)
{}

JointEffortController::~JointEffortController()
{
  sub_command_.shutdown();
}

bool JointEffortController::init(hardware_interface::PositionVelocityTorqueGainsJointInterface *robot, 
  const std::string &joint_name)
{
  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  return true;
}

bool JointEffortController::init(hardware_interface::PositionVelocityTorqueGainsJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointEffortController::setCommandCB, this);

  return true;
}

std::string JointEffortController::getJointName()
{
  return joint_.getName();
}

// Set the joint velocity command
void JointEffortController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void JointEffortController::getCommand(double& cmd)
{
  cmd = command_;
}

void JointEffortController::starting(const ros::Time& time)
{
  joint_.setCommandPosition(0.);
  joint_.setCommandVelocity(0.);
  joint_.setCommandKp(0.);
  joint_.setCommandKd(0.);

  command_ = joint_.getPosition();
  //pid_controller_.reset();
}

void JointEffortController::update(const ros::Time& time, const ros::Duration& period)
{

  joint_.setCommandEffort(command_);

}

void JointEffortController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(position_velocity_torque_gains_controllers::JointEffortController, controller_interface::ControllerBase)
