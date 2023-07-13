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

#include <memory>
#include <string>

#include <pix_sweeping_driver/control_converter.hpp>

namespace pix_sweeping_driver
{
namespace control_converter
{

ControlConverter::ControlConverter() : Node("control_converter")
{
  // ros params
  param_.autoware_control_command_timeout =
    declare_parameter("autoware_control_command_timeout", 100);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);
  param_.max_steering_angle = declare_parameter("max_steering_angle", 0.5236);
  param_.steering_factor = 500.0 / param_.max_steering_angle;

  // initialization engage
  engage_cmd_ = false;

  // initialize msgs and timestamps
  gear_report_received_time_ = this->now();
  actuation_command_received_time_ = this->now();
  gear_command_received_time_ = this->now();

  // publishers
  brake_command_pub_ =
    create_publisher<BrakeCommand>("/pix_sweeping/brake_command", rclcpp::QoS(1));
  throttle_command_pub_ =
    create_publisher<ThrottleCommand>("/pix_sweeping/throttle_command", rclcpp::QoS(1));
  steering_command_pub_ =
    create_publisher<SteeringCommand>("/pix_sweeping/steering_command", rclcpp::QoS(1));
  vehicle_mode_command_pub_ =
    create_publisher<VehicleModeCommand>("/pix_sweeping/vehicle_mode_command", rclcpp::QoS(1));
  gear_command_pub_ =
    create_publisher<GearCommand>("/pix_sweeping/gear_command", rclcpp::QoS(1));
  park_command_pub_ =
    create_publisher<ParkCommand>("/pix_sweeping/park_command", rclcpp::QoS(1));
  
  //services
  control_mode_server_ = create_service<autoware_auto_vehicle_msgs::srv::ControlModeCommand>(
    "/control/control_mode_request",
    std::bind(
      &ControlConverter::onControlModeRequest, this, std::placeholders::_1, std::placeholders::_2));

  // subscribers
  actuation_command_sub_ =
    create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
      "/control/command/actuation_cmd", 1,
      std::bind(&ControlConverter::callbackActuationCommand, this, std::placeholders::_1));
  gear_command_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1,
    std::bind(&ControlConverter::callbackGearCommand, this, std::placeholders::_1));
  gear_report_sub_ = create_subscription<GearReport>(
    "/pix_sweeping/gear_report", 1,
    std::bind(&ControlConverter::callbackGearReport, this, std::placeholders::_1));

  // operation mode
  operation_mode_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", 1,
    std::bind(&ControlConverter::callbackOperationMode, this, std::placeholders::_1));
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
    std::bind(&ControlConverter::timerCallback, this));
}

void ControlConverter::callbackActuationCommand(
  const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr & msg)
{
  actuation_command_received_time_ = this->now();
  actuation_command_ptr_ = msg;
}

void ControlConverter::callbackGearCommand(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr & msg)
{
  gear_command_received_time_ = this->now();
  gear_command_ptr_ = msg;
}

void ControlConverter::callbackGearReport(const GearReport::ConstSharedPtr & msg)
{
  gear_report_received_time_ = this->now();
  gear_sta_fb_ptr_ = msg;
}

void ControlConverter::callbackOperationMode(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr & msg)
 {
   operation_mode_ptr_ = msg;
 }

void ControlConverter::onControlModeRequest(
  const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
  const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
{
  if (request->mode == autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS) {
    engage_cmd_ = true;
    response->success = true;
    return;
  }

  if (request->mode == autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL) {
    engage_cmd_ = false;
    response->success = true;
    return;
  }

  RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
  response->success = false;
  return;
}

void ControlConverter::timerCallback()
{
  const rclcpp::Time current_time = this->now();
  const double actuation_command_delta_time_ms =
    (current_time - actuation_command_received_time_).seconds() * 1000.0;
  const double gear_command_delta_time_ms =
    (current_time - gear_command_received_time_).seconds() * 1000.0;
  const double drive_sta_fb_delta_time_ms =
    (current_time - gear_report_received_time_).seconds() * 1000.0;

  if(actuation_command_delta_time_ms > param_.autoware_control_command_timeout
      || gear_command_delta_time_ms > param_.autoware_control_command_timeout
      || actuation_command_ptr_==nullptr
      || gear_command_ptr_==nullptr
      || operation_mode_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "actuation command timeout = %f ms, gear command timeout = %f",
      actuation_command_delta_time_ms, gear_command_delta_time_ms);
    return;
  }
  if(drive_sta_fb_delta_time_ms > param_.autoware_control_command_timeout || gear_sta_fb_ptr_ == nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "gear feedback timeout = %f ms.", drive_sta_fb_delta_time_ms);
    return;
  }

  // sending control messages to pix dirver control command
  BrakeCommand brake_command_msg;
  ThrottleCommand throttle_command_msg;
  GearCommand gear_command_msg;
  SteeringCommand steer_command_msg;
  VehicleModeCommand vehicle_mode_command_msg;
  ParkCommand park_command_msg;

  // parking 
  park_command_msg.park_en_ctrl = true;
  park_command_msg.header.stamp = current_time;

  // brake
  brake_command_msg.header.stamp = current_time;
  brake_command_msg.brake_pedal_target = actuation_command_ptr_->actuation.brake_cmd * 100.0;
  brake_command_msg.brake_en_ctrl = true;

  // steer
  steer_command_msg.header.stamp = current_time;
  steer_command_msg.steer_angle_speed = 250;
  steer_command_msg.steer_angle_target =
    -actuation_command_ptr_->actuation.steer_cmd * param_.steering_factor;
  steer_command_msg.steer_en_ctrl = true;

  // gear
  gear_command_msg.header.stamp = current_time;
  gear_command_msg.gear_en_ctrl = true; 
  switch (gear_command_ptr_->command) {
    case autoware_auto_vehicle_msgs::msg::GearCommand::NONE:
      gear_command_msg.gear_target = static_cast<int8_t>(GearCommand::INVALID_1);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
      gear_command_msg.gear_target = static_cast<int8_t>(GearCommand::DRIVE_1);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL:
      gear_command_msg.gear_target = static_cast<int8_t>(GearCommand::NEUTRAL_1);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
      gear_command_msg.gear_target = static_cast<int8_t>(GearCommand::REVERSE_1);
      break;
    default:
      gear_command_msg.gear_target = static_cast<int8_t>(GearCommand::INVALID_1);
      break;
  }

  // vehicle mode 
  vehicle_mode_command_msg.steer_mode_ctrl =
    static_cast<int8_t>(VehicleModeCommand::NON_DIRECTION_STEER_3);
  vehicle_mode_command_msg.drive_mode_ctrl = 
    static_cast<int8_t>(VehicleModeCommand::THROTTLE_PADDLE_DRIVE_2);
  
  // throttle
  // if(engage_cmd_)
  // {
  //   throttle_command_msg.acu_chassis_driver_en_ctrl = ACU_CHASSISDRIVERENCTRL_ENABLE;
  // }else{
  //   throttle_command_msg.acu_chassis_driver_en_ctrl = ACU_CHASSISDRIVERENCTRL_DISABLE;
  // }
  throttle_command_msg.dirve_throttle_pedal_target = actuation_command_ptr_->actuation.accel_cmd * 100.0;
  throttle_command_msg.dirve_en_ctrl = true;


  // keep shifting and braking when target gear is different from actual gear
  if (gear_sta_fb_ptr_->gear_actual != gear_command_msg.gear_target) {
    brake_command_msg.brake_pedal_target = 20.0;
    throttle_command_msg.dirve_throttle_pedal_target = 0.0;
  }

  // check the opeartion mode, if the operation mode is STOP, it should triger the parking brake
  if(operation_mode_ptr_->mode == operation_mode_ptr_->STOP){
    park_command_msg.park_target = true;
  }else{
    park_command_msg.park_target = false;
  }

  // publishing msgs
  brake_command_pub_->publish(brake_command_msg);
  throttle_command_pub_->publish(throttle_command_msg);
  steering_command_pub_->publish(steer_command_msg);
  gear_command_pub_->publish(gear_command_msg);
  park_command_pub_->publish(park_command_msg);
  vehicle_mode_command_pub_->publish(vehicle_mode_command_msg);
}
} // namespace control_converter
} // namespace pix_driver
