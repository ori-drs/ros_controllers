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

#include <effort_controllers/joint_group_position_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

namespace effort_controllers
{

/**
 * \brief Position controller (PID) for a set of effort controlled joints (torque or force).
 *
 * This class performs PID control on position commands and outputs efforts for a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupPositionController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The desired joint positions
 */
  JointGroupPositionController::JointGroupPositionController() {}
  JointGroupPositionController::~JointGroupPositionController() {sub_command_.shutdown();}

  bool JointGroupPositionController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
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
    controller_state_publishers_.resize(n_joints_);

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

      // Set up controller state publishers
      controller_state_publishers_[i].reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, joint_name + "/state", 10));
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    pub_cmd_republisher_.init(n, "command_reference", 10);
    {
      std::lock_guard<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> lock(pub_cmd_republisher_);
      pub_cmd_republisher_.msg_.data.resize(n_joints_);
    }

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointGroupPositionController::commandCB, this);
    return true;
  }

  void JointGroupPositionController::starting(const ros::Time& time)
  {
    std::vector<double> current_positions(n_joints_, 0.0);
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      current_positions[i] = joints_[i].getPosition();
      enforceJointLimits(current_positions[i], i);
      pid_controllers_[i].reset();
    }
    commands_buffer_.initRT(current_positions);
  }

  void JointGroupPositionController::update(const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> & commands = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {
        double command_position = commands[i];

        double error;
        double commanded_effort;

        double current_position = joints_[i].getPosition();

        // Make sure joint is within limits if applicable
        enforceJointLimits(command_position, i);

        // Compute position error
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
        {
          angles::shortest_angular_distance_with_large_limits(
            current_position,
            command_position,
            joint_urdfs_[i]->limits->lower,
            joint_urdfs_[i]->limits->upper,
            error);
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
          error = angles::shortest_angular_distance(current_position, command_position);
        }
        else //prismatic
        {
          error = command_position - current_position;
        }

        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        commanded_effort = pid_controllers_[i].computeCommand(error, period);

        joints_[i].setCommand(commanded_effort);

        // publish state
        // TODO: Parametrise controller state publishing
        if (loop_count_ % 1 == 0 && controller_state_publishers_[i]->trylock())
        {
          controller_state_publishers_[i]->msg_.header.stamp = time;
          controller_state_publishers_[i]->msg_.set_point = command_position;
          controller_state_publishers_[i]->msg_.process_value = current_position;
          controller_state_publishers_[i]->msg_.process_value_dot = joints_[i].getVelocity();
          controller_state_publishers_[i]->msg_.error = error;
          controller_state_publishers_[i]->msg_.time_step = period.toSec();
          controller_state_publishers_[i]->msg_.command = commanded_effort;

          double dummy;
          bool antiwindup;
          pid_controllers_[i].getGains(controller_state_publishers_[i]->msg_.p,
            controller_state_publishers_[i]->msg_.i,
            controller_state_publishers_[i]->msg_.d,
            controller_state_publishers_[i]->msg_.i_clamp,
            dummy,
            antiwindup);
          controller_state_publishers_[i]->msg_.antiwindup = static_cast<char>(antiwindup);
          controller_state_publishers_[i]->unlockAndPublish();
        }
    }

    // Republish set-point
    if (pub_cmd_republisher_.trylock())
    {
      pub_cmd_republisher_.msg_.data = commands;
      pub_cmd_republisher_.unlockAndPublish();
    }

    // update counter for state publishing
    loop_count_++;
  }

  void JointGroupPositionController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    commands_buffer_.writeFromNonRT(msg->data);
  }

  void JointGroupPositionController::enforceJointLimits(double &command, unsigned int index)
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
      {
        command = joint_urdfs_[index]->limits->upper;
      }
      else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        command = joint_urdfs_[index]->limits->lower;
      }
    }
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( effort_controllers::JointGroupPositionController, controller_interface::ControllerBase)
