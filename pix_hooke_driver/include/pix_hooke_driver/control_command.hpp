// Copyright 2023 Pixmoving, Inc. 
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_
#define PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <can_msgs/msg/frame.hpp>

#include <pix_hooke_driver_msgs/msg/a2v_brake_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_drive_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_steer_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_vehicle_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_wheel_ctrl.hpp>

#include <pix_hooke_driver/a2v_brakectrl_131.hpp>
#include <pix_hooke_driver/a2v_drivectrl_130.hpp>
#include <pix_hooke_driver/a2v_steerctrl_132.hpp>
#include <pix_hooke_driver/a2v_vehiclectrl_133.hpp>
#include <pix_hooke_driver/a2v_wheelctrl_135.hpp>

#include <string>
#include <memory>

namespace pix_hooke_driver
{
namespace control_command
{
using A2vBrakeCtrl = pix_hooke_driver_msgs::msg::A2vBrakeCtrl;
using A2vDriveCtrl = pix_hooke_driver_msgs::msg::A2vDriveCtrl;
using A2vSteerCtrl = pix_hooke_driver_msgs::msg::A2vSteerCtrl;
using A2vWheelCtrl = pix_hooke_driver_msgs::msg::A2vWheelCtrl;
using A2vVehicleCtrl = pix_hooke_driver_msgs::msg::A2vVehicleCtrl;

/**
 * @brief param structure of control command node
 * @param base_frame_id frame id of vehicle
 * @param loop_rate loop rate of publishers in hz
 * @param command_timeout_ms timeout threshold of control command msg from control converter in ms
 */
struct Param
{
  std::string base_frame_id;
  double loop_rate;
  int command_timeout_ms;
};

class ControlCommand : public rclcpp::Node
{
private:
  // parameter
  Param param_;

  // subscribers from pix driver autoware interface
  rclcpp::Subscription<A2vBrakeCtrl>::SharedPtr a2v_brake_ctrl_sub_;
  rclcpp::Subscription<A2vDriveCtrl>::SharedPtr a2v_drive_ctrl_sub_;
  rclcpp::Subscription<A2vSteerCtrl>::SharedPtr a2v_steer_ctrl_sub_;
  rclcpp::Subscription<A2vWheelCtrl>::SharedPtr a2v_wheel_ctrl_sub_;
  rclcpp::Subscription<A2vVehicleCtrl>::SharedPtr a2v_vehicle_ctrl_sub_;

  A2vBrakeCtrl::ConstSharedPtr brake_ctrl_ptr_;
  A2vDriveCtrl::ConstSharedPtr drive_ctrl_ptr_;
  A2vSteerCtrl::ConstSharedPtr steer_ctrl_ptr_;
  A2vWheelCtrl::ConstSharedPtr wheel_ctrl_ptr_;
  A2vVehicleCtrl::ConstSharedPtr vehicle_ctrl_ptr_;

  // control command structures
  A2vdrivectrl130 a2v_drivectrl_130_entity_;
  A2vbrakectrl131 a2v_brakectrl_131_entity_;
  A2vsteerctrl132 a2v_steerctrl_132_entity_;
  A2vvehiclectrl133 a2v_vehiclectrl_133_entity_;
  A2vwheelctrl135 a2v_wheelctrl_135_entity_;

  // msg received timestamp
  rclcpp::Time brake_command_received_time_;
  rclcpp::Time drive_command_received_time_;
  rclcpp::Time steer_command_received_time_;
  rclcpp::Time wheel_command_received_time_;
  rclcpp::Time vehicle_command_received_time_;

  // state control
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_ctrl_sub_;
  bool is_engage_;

  // publishers to can card driver
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  // publishing can msgs
  can_msgs::msg::Frame::ConstSharedPtr brake_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr drive_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr steer_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr wheel_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr vehicle_ctrl_can_ptr_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ControlCommand();
  /**
   * @brief callback function of A2vBrakeCtrl msg, to store the data to member variable
   * 
   * @param msg 
   */
  void callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg);
  /**
   * @brief callback function of A2vDriveCtrl msg, to store the data to member variable
   * 
   * @param msg 
   */
  void callbackDriveCtrl(const A2vDriveCtrl::ConstSharedPtr & msg);
  /**
   * @brief callback function of A2vSteerCtrl msg, to store the data to member variable
   * 
   * @param msg 
   */
  void callbackSteerCtrl(const A2vSteerCtrl::ConstSharedPtr & msg);
  /**
   * @brief callback function of A2vWheelCtrl msg, to store the data to member variable
   * 
   * @param msg 
   */
  void callbackWheelCtrl(const A2vWheelCtrl::ConstSharedPtr & msg);
  /**
   * @brief callback function of A2vVehicleCtrl msg, to store the data to member variable
   * 
   * @param msg 
   */
  void callbackVehicleCtrl(const A2vVehicleCtrl::ConstSharedPtr & msg);
  /**
   * @brief callback function of Bool msg, to decide publish can frame or not
   * 
   * @param msg 
   */
  void callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  /**
   * @brief timer callback function, convert pix_hooke_driver msgs to canbus Frames, than publish them
   * 
   */
  void timerCallback();
};
} // namespace control_command
} // namespace pix_hooke_driver
#endif // PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_