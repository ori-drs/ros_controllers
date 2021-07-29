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

#pragma once


#include <ros/node_handle.h>
#include <hardware_interface/position_velocity_torque_gains_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/position_velocity_torque_gains_command_interface.h>

namespace position_velocity_torque_gains_controllers
{

/**
 * \brief Single joint controller.
 *
 * This class passes the commanded position signal down to the joint Position, Velocity, Torques, Gains.
 *
 * \section ROS interface
 *
 * \param type hardware interface type.
 * \param joint Name of the joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint command to apply.
 */
class JointPositionController: public controller_interface::Controller<hardware_interface::PositionVelocityTorqueGainsJointInterface>
{
public:
  JointPositionController(); 
  ~JointPositionController();

  bool init(hardware_interface::PositionVelocityTorqueGainsJointInterface* hw, ros::NodeHandle &n);

  void reloadGains(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void starting(const ros::Time& time);

  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  hardware_interface::PositionVelocityTorqueGainsJointHandle joint_;
  realtime_tools::RealtimeBuffer<double> command_buffer_;

private:
  ros::Subscriber sub_command_;
  control_toolbox::Pid pid_controller_;
  ros::ServiceServer ros_server_service_reload_gains_;
  void commandCB(const std_msgs::Float64ConstPtr& msg);
};

}
