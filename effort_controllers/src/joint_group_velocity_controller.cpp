/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  Copyright (c) 2021, University of Oxford
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

#include <effort_controllers/joint_group_velocity_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

namespace effort_controllers
{

/**
 * \brief Velocity controller (PID) for a set of effort controlled joints (torque or force).
 *
 * This class performs PID control on velocity commands and outputs efforts for a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupVelocityController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The desired joint velocities
 */
  JointGroupVelocityController::JointGroupVelocityController() {}
  JointGroupVelocityController::~JointGroupVelocityController() {sub_command_.shutdown();}

  bool JointGroupVelocityController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    if(!n.getParam("joints", joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'joints' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParamWithNodeHandle("robot_description", n))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    pid_controllers_.resize(n_joints_);

    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];

      try
      {
        joints_.push_back(hw->getHandle(joint_name));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);

      // Load PID Controller using gains set on parameter server
      if (!pid_controllers_[i].init(ros::NodeHandle(n, joint_name + "/pid")))
      {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_name + "/pid");
        return false;
      }
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointGroupVelocityController::commandCB, this);
    return true;
  }

  void JointGroupVelocityController::starting(const ros::Time& time)
  {
    std::vector<double> zero_velocities(n_joints_, 0.0);
    commands_buffer_.initRT(zero_velocities);
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      pid_controllers_[i].reset();
    }
  }

  void JointGroupVelocityController::update(const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> & commands = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {
        double error = commands[i] - joints_[i].getVelocity();
        
        // Enforce joint velocity limit
        const double& velocity_limit = joint_urdfs_[i]->limits->velocity;
        error = std::max(-velocity_limit, std::min(error, velocity_limit));

        // TODO: Enforce joint position limits (i.e., do not command further into position limit)

        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        double commanded_effort = pid_controllers_[i].computeCommand(error, period);

        joints_[i].setCommand(commanded_effort);
    }
  }

  void JointGroupVelocityController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    commands_buffer_.writeFromNonRT(msg->data);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS(effort_controllers::JointGroupVelocityController, controller_interface::ControllerBase)
