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

#ifndef PIX_HOOKE_DRIVER__CONTROL_CONVERTER_HPP_
#define PIX_HOOKE_DRIVER__CONTROL_CONVERTER_HPP_

#include <memory>
#include <string>
//ros
#include <rclcpp/rclcpp.hpp>
// autoware
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
// pix control
#include <pix_hooke_driver_msgs/msg/a2v_brake_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_drive_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_steer_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_vehicle_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_wheel_ctrl.hpp>
// pix report
#include <pix_hooke_driver_msgs/msg/v2a_drive_sta_fb.hpp>

namespace pix_hooke_driver
{
namespace control_converter
{
using A2vBrakeCtrl = pix_hooke_driver_msgs::msg::A2vBrakeCtrl;
using A2vDriveCtrl = pix_hooke_driver_msgs::msg::A2vDriveCtrl;
using A2vSteerCtrl = pix_hooke_driver_msgs::msg::A2vSteerCtrl;
using A2vWheelCtrl = pix_hooke_driver_msgs::msg::A2vWheelCtrl;
using A2vVehicleCtrl = pix_hooke_driver_msgs::msg::A2vVehicleCtrl;
using V2aDriveStaFb = pix_hooke_driver_msgs::msg::V2aDriveStaFb;

//chassis driver enable control
enum { ACU_CHASSISDRIVERENCTRL_DISABLE, ACU_CHASSISDRIVERENCTRL_ENABLE };
//chassis drive mode control
enum {
  ACU_CHASSISDRIVERMODECTRL_SPEED_CTRL_MODE,
  ACU_CHASSISDRIVERMODECTRL_THROTTLE_CTRL_MODE,
  ACU_CHASSISDRIVERMODECTRL_RESERVE,
};
// chassis gear control
enum {
  ACU_CHASSISGEARCTRL_DEFAULT_N,
  ACU_CHASSISGEARCTRL_D,
  ACU_CHASSISGEARCTRL_N,
  ACU_CHASSISGEARCTRL_R
};
// chassis gear feedback
enum { VCU_CHASSISGEARFB_NO_USE, VCU_CHASSISGEARFB_D, VCU_CHASSISGEARFB_N, VCU_CHASSISGEARFB_R };
// chassi steer mode control
enum {
  ACU_CHASSISSTEERMODECTRL_FRONT_ACKERMAN = 0,
  ACU_CHASSISSTEERMODECTRL_SAME_FRONT_AND_BACK,
  ACU_CHASSISSTEERMODECTRL_FRONT_DIFFERENT_BACK,
  ACU_CHASSISSTEERMODECTRL_BACK_ACKRMAN,
  ACU_CHASSISSTEERMODECTRL_FRONT_BACK
};

/**
 * @brief node parameter
 * @param loop_rate loop rate of publisher
 * @param max_steerig_angle max steering angle in radians
 * @param steering_factor the rate to convert steering angle to steering command signal value
 * @param autoware_control_command_timeout control command timeout threshold in ms
 */
struct Param
{
  double loop_rate;                     // hz
  double max_steering_angle;            // radians
  double steering_factor;               //
  int autoware_control_command_timeout; // ms
};

class ControlConverter : public rclcpp::Node
{
private:
  // parameters
  Param param_;
  bool engage_cmd_;

  // shared msgs
  V2aDriveStaFb::ConstSharedPtr drive_sta_fb_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_command_ptr_;
  tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr actuation_command_ptr_;
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_ptr_;

  // timestamps
  rclcpp::Time drive_sta_fb_received_time_;
  rclcpp::Time gear_command_received_time_;
  rclcpp::Time actuation_command_received_time_;

  // subscribers
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::ConstSharedPtr
    actuation_command_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::ConstSharedPtr
    gear_command_sub_;
  rclcpp::Subscription<pix_hooke_driver_msgs::msg::V2aDriveStaFb>::ConstSharedPtr
    drive_feedback_sub_;
  // need to be done
  // emergency command
  // hazard lights command
  // turn indicators command
  // operation mode
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::ConstSharedPtr
    operation_mode_sub_;

  // services
  rclcpp::Service<autoware_auto_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_server_;

  // publishers
  rclcpp::Publisher<A2vBrakeCtrl>::SharedPtr a2v_brake_ctrl_pub_;
  rclcpp::Publisher<A2vDriveCtrl>::SharedPtr a2v_drive_ctrl_pub_;
  rclcpp::Publisher<A2vSteerCtrl>::SharedPtr a2v_steer_ctrl_pub_;
  rclcpp::Publisher<A2vVehicleCtrl>::SharedPtr a2v_vehicle_ctrl_pub_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  /**
   * @brief Construct a new Control Converter object
   * 
   */
  ControlConverter();
  /**
   * @brief callback function of actuation command, in order to get accel pedal, brake pedal, steer command
   * 
   * @param msg input message
   */
  void callbackActuationCommand(
    const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr & msg);
  /**
   * @brief callback function of actuation command, in order to get the gear command
   * 
   * @param msg input message
   */
  void callbackGearCommand(
    const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr & msg);
  /**
   * @brief callback function of v2a drive status feedback, in order to get the current gear of vehicle
   * 
   * @param msg input message
   */
  void callbackDriveStatusFeedback(
    const pix_hooke_driver_msgs::msg::V2aDriveStaFb::ConstSharedPtr & msg);
  /**
   * @brief callback function of operation mode feedback
   * 
   * @param msg input message
   */
  void callbackOperationMode(
     const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr & msg);
  /**
   * @brief request function to modify control mode AUTO/MANUAL
   * 
   * @param request 
   * @param response 
   */
  void onControlModeRequest(
    const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
    const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response);
  /**
   * @brief timer callback function, to evaluate whether if msgs are timeout, than publish control msgs to pix driver control command node
   * 
   */
  void timerCallback();
};

} // namespace control_converter
} // namespace pix_hooke_driver

#endif // PIX_HOOKE_DRIVER__CONTROL_CONVERTER_HPP_