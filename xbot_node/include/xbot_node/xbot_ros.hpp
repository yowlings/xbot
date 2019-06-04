/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file /xbot_node/include/xbot_node/xbot_ros.hpp
 *
 * @brief Wraps the xbot driver in a ROS-specific library
 *
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef XBOT_ROS_HPP_
#define XBOT_ROS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <angles/angles.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16MultiArray.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <ecl/sigslots.hpp>
#include <ecl/threads.hpp>
#include <xbot_msgs/CoreSensor.h>
#include <xbot_msgs/ExtraSensor.h>
#include <xbot_msgs/Echo.h>
#include <xbot_msgs/InfraRed.h>
#include <xbot_msgs/Battery.h>


#include <xbot_driver/xbot.hpp>
#include <xbot_msgs/XbotState.h>
#include <xbot_msgs/RawImu.h>

#include <geometry_msgs/Quaternion.h>
#include "odometry.hpp"


#include <xbot_talker/play.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{
class XbotRos
{
public:
  XbotRos(std::string& node_name);
  ~XbotRos();
  bool init(ros::NodeHandle& nh);
  bool update();
  void call_srv();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  /*********************
   ** Variables
   **********************/
  std::string name; // name of the ROS node
  Xbot xbot;
  sensor_msgs::JointState joint_states;
  Odometry odometry;
  bool cmd_vel_timed_out_; // stops warning spam when cmd_vel flags as timed out more than once in a row
  bool base_serial_timed_out_; // stops warning spam when serial connection timed out more than once in a row
  bool sensor_serial_timed_out_;

  bool led_indicate_battery;
  bool announced_battery;
  ros::ServiceClient srv_play;
  ecl::Thread client_thread;


  /*********************
   ** Ros Publishers
   **********************/
  ros::Publisher core_sensor_publisher;
  ros::Publisher extra_sensor_publisher;
  ros::Publisher yaw_platform_state_publisher;
  ros::Publisher pitch_platform_state_publisher;
  ros::Publisher battery_state_publisher;
  ros::Publisher stop_buttom_state_publisher;
  ros::Publisher sound_state_publisher;
  ros::Publisher imu_data_publisher;
  ros::Publisher raw_imu_data_publisher;
  ros::Publisher infrared_data_publisher;
  ros::Publisher echo_data_publisher;
  ros::Publisher joint_state_publisher;
  ros::Publisher robot_state_publisher;


  /*********************
   ** Ros Subscribers
   **********************/
  ros::Subscriber motor_enable_command_subscriber;
  ros::Subscriber velocity_command_subscriber;
  ros::Subscriber yaw_platform_command_subscriber;
  ros::Subscriber pitch_platform_command_subscriber;
  ros::Subscriber sound_command_subscriber;
  ros::Subscriber led_command_subscriber;
  ros::Subscriber lift_command_subscirber;
  ros::Subscriber reset_odometry_subscriber;

  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);

  /*********************
  ** Ros Callbacks
  **********************/
  void subscribeMotorEnableCommand(const std_msgs::Bool);
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);
  void subscribeYawPlatformCommand(const std_msgs::Int8);
  void subscribePitchPlatformCommand(const std_msgs::Int8);
  void subscribeSoundCommand(const std_msgs::Bool);
  void subscribeLedCommand(const std_msgs::UInt8);
  void subscribeLiftCommand(const std_msgs::UInt8);
  void subscribeResetOdometry(const std_msgs::EmptyConstPtr);

  /*********************
   ** SigSlots
   **********************/
  ecl::Slot<> base_slot_stream_data;
  ecl::Slot<> sensor_slot_stream_data;

  /*********************
   ** Base Slot Callbacks
   **********************/
  void processBaseStreamData();
  void publishWheelState();
  void publishCoreSensor();
  void publishEchoData();
  void publishInfraredData();
  void publishBatteryState();
  void publishStopButtonState();


  /*********************
   ** Sensor Slot Callbacks
   **********************/
  void processSensorStreamData();
  void publishExtraSensor();
  void publishYawPlatformState();
  void publishPitchPlatformState();
  void publishSoundState();
  void publishInertia();
  void publishRawInertia();
  void publishRobotState();



};

} // namespace xbot

#endif /* XBOT_ROS_HPP_ */
