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

#include <pix_robobus_driver/control_converter.hpp>

namespace pix_robobus_driver
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
  param_.steering_factor = 450.0 / param_.max_steering_angle;

  // initialization engage
  engage_cmd_ = false;

  // initialize msgs and timestamps
  gear_report_received_time_ = this->now();
  actuation_command_received_time_ = this->now();
  gear_command_received_time_ = this->now();

  // topic name 
  std::string throttle_ctrl_pub_topic_name = "/pix_robobus/throttle_command";
  std::string gear_ctrl_pub_topic_name = "/pix_robobus/gear_command";
  std::string steer_ctrl_pub_topic_name = "/pix_robobus/steering_command";
  std::string brake_ctrl_pub_topic_name = "/pix_robobus/brake_command";
  std::string park_ctrl_pub_topic_name = "/pix_robobus/park_command";
  std::string vehicle_ctrl_pub_topic_name = "/pix_robobus/vehicle_mode_command";
  // publishers
  throttle_ctrl_pub_ =
    create_publisher<ThrottleCommand>(throttle_ctrl_pub_topic_name, rclcpp::QoS(1));
  gear_ctrl_pub_ =
    create_publisher<GearCommand>(gear_ctrl_pub_topic_name, rclcpp::QoS(1));
  steer_ctrl_pub_ =
    create_publisher<SteeringCommand>(steer_ctrl_pub_topic_name, rclcpp::QoS(1));
  brake_ctrl_pub_ =
    create_publisher<BrakeCommand>(brake_ctrl_pub_topic_name, rclcpp::QoS(1));
  park_ctrl_pub_ =
    create_publisher<ParkCommand>(park_ctrl_pub_topic_name, rclcpp::QoS(1));
  vehicle_ctrl_pub_ =
    create_publisher<VehicleModeCommand>(vehicle_ctrl_pub_topic_name, rclcpp::QoS(1));
  
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
  gear_feedback_sub_ = create_subscription<GearReport>(
    "/pix_robobus/gear_report", 1,
    std::bind(&ControlConverter::callbackGearReport, this, std::placeholders::_1));

  // operation mode
  operation_mode_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", 1,
    std::bind(&ControlConverter::callbackOperationMode, this, std::placeholders::_1));
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
    std::bind(&ControlConverter::timerCallback, this));

  //remote control require
  auto_remote_ctrl_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg>("/pix_robobus/auto_remote_ctrl_msg", 1, std::bind(&ControlConverter::callbackAutoRemoteControlCommand, this, std::placeholders::_1));
  vcu_report_sub_ = create_subscription<pix_robobus_driver_msgs::msg::VcuReport>( "/pix_robobus/vcu_report", 1, std::bind(&ControlConverter::callbackVcuReport, this, std::placeholders::_1));
}

void ControlConverter::callbackVcuReport(
  const pix_robobus_driver_msgs::msg::VcuReport::ConstSharedPtr & msg)
{
  current_velocity = msg->vehicle_speed;
}


void ControlConverter::callbackAutoRemoteControlCommand(
  const pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg::ConstSharedPtr & msg)
{
  remote_require = msg->auto_remote_drive_ctrl_mode;
}

void ControlConverter::callbackActuationCommand(
  const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr & msg)
{
  actuation_command_received_time_ = this->now();
  actuation_command_ptr_ = msg;
  // RCLCPP_INFO(get_logger(), "callbackActuationCommand: %f", actuation_command_received_time_.seconds() * 1000.0);
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
  gear_report_ptr_ = msg;
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
  if(drive_sta_fb_delta_time_ms > param_.autoware_control_command_timeout || gear_report_ptr_ == nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "gear feedback timeout = %f ms.", drive_sta_fb_delta_time_ms);
    return;
  }

  // sending control messages to pix dirver control command
  BrakeCommand brake_ctrl_msg;
  ThrottleCommand throttle_ctrl_msg;
  SteeringCommand steer_ctrl_msg;
  VehicleModeCommand vehicle_ctrl_msg;
  GearCommand gear_ctrl_msg;
  ParkCommand park_ctrl_msg;

  if (remote_require == 0)
  {
    // brake
    brake_ctrl_msg.header.stamp = current_time;
    brake_ctrl_msg.brake_pedal_target = actuation_command_ptr_->actuation.brake_cmd * 100.0;
    brake_ctrl_msg.brake_en_ctrl = 1;

    // steer
    steer_ctrl_msg.header.stamp = current_time;
    steer_ctrl_msg.steer_angle_speed = 250;
    steer_ctrl_msg.steer_angle_target =
    actuation_command_ptr_->actuation.steer_cmd * param_.steering_factor;
    steer_ctrl_msg.steer_en_ctrl = 1;

    

    // gear
    gear_ctrl_msg.header.stamp = current_time;
    gear_ctrl_msg.gear_en_ctrl = 1;
    switch (gear_command_ptr_->command) {
      case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
        gear_ctrl_msg.gear_target  = static_cast<int8_t>(GEAR_DRIVE);
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL:
        gear_ctrl_msg.gear_target  = static_cast<int8_t>(GEAR_NEUTRAL);
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
        gear_ctrl_msg.gear_target  = static_cast<int8_t>(GEAR_REVERSE);
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::PARK:
        gear_ctrl_msg.gear_target = static_cast<int8_t>(GEAR_PARK);
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::NONE :
        gear_ctrl_msg.gear_target = static_cast<int8_t>(GEAR_INVALID);
        break;
      default:
        gear_ctrl_msg.gear_target  = static_cast<int8_t>(GEAR_NEUTRAL);
        break;
    }
    // throttle
    throttle_ctrl_msg.header.stamp = current_time;
    throttle_ctrl_msg.dirve_en_ctrl = 1;
    throttle_ctrl_msg.dirve_throttle_pedal_target = actuation_command_ptr_->actuation.accel_cmd * 100.0;

    // vehicle
    vehicle_ctrl_msg.header.stamp = current_time;
    vehicle_ctrl_msg.steer_mode_ctrl = static_cast<int8_t>(STEER_NON_DIRECTION);
    vehicle_ctrl_msg.drive_mode_ctrl = DIRVE_ENCTRL_THROTTLE_PADDLE;
    

    // keep shifting and braking when target gear is different from actual gear
    if (gear_report_ptr_->gear_actual != gear_ctrl_msg.gear_target ) {
      brake_ctrl_msg.brake_pedal_target = 5.0;
      throttle_ctrl_msg.dirve_throttle_pedal_target = 0.0;
    }

    // parking
    park_ctrl_msg.header.stamp = current_time;
    park_ctrl_msg.park_en_ctrl = true;
    if(operation_mode_ptr_->mode == operation_mode_ptr_->STOP){
      park_ctrl_msg.park_target = true;
    } else{
      park_ctrl_msg.park_target = false;
    }
    parking_brake = 0;
    RCLCPP_INFO(get_logger(), "remote_control mode 0");
  }
  else if (remote_require == 1)
  {
    if (parking_brake < 20)
      parking_brake += 0.1;


    // steer
    steer_ctrl_msg.header.stamp = current_time;
    steer_ctrl_msg.steer_angle_speed = 250;
    steer_ctrl_msg.steer_angle_target = 0;
    steer_ctrl_msg.steer_en_ctrl = 1;

    // gear
    gear_ctrl_msg.header.stamp = current_time;
    gear_ctrl_msg.gear_en_ctrl = 1;
    gear_ctrl_msg.gear_target  = static_cast<int8_t>(GEAR_NEUTRAL);

    // throttle
    throttle_ctrl_msg.header.stamp = current_time;
    throttle_ctrl_msg.dirve_en_ctrl = 1;
    throttle_ctrl_msg.dirve_throttle_pedal_target = 0;

    // vehicle
    vehicle_ctrl_msg.header.stamp = current_time;
    vehicle_ctrl_msg.steer_mode_ctrl = static_cast<int8_t>(STEER_NON_DIRECTION);
    vehicle_ctrl_msg.drive_mode_ctrl = DIRVE_ENCTRL_THROTTLE_PADDLE;
    

    // parking
    park_ctrl_msg.header.stamp = current_time;
    park_ctrl_msg.park_en_ctrl = true;
    if(current_velocity == 0){
      park_ctrl_msg.park_target = true;
      // brake
      parking_brake = 0;
      brake_ctrl_msg.header.stamp = current_time;
      brake_ctrl_msg.brake_pedal_target = parking_brake;
      brake_ctrl_msg.brake_en_ctrl = 1;
    } else{
      park_ctrl_msg.park_target = false;
      // brake
      brake_ctrl_msg.header.stamp = current_time;
      brake_ctrl_msg.brake_pedal_target = parking_brake;
      brake_ctrl_msg.brake_en_ctrl = 1;
    }
    RCLCPP_INFO(get_logger(), "remote_control mode 1");
  }

  // publishing msgs
  throttle_ctrl_pub_->publish(throttle_ctrl_msg);
  gear_ctrl_pub_->publish(gear_ctrl_msg);
  steer_ctrl_pub_->publish(steer_ctrl_msg);
  brake_ctrl_pub_->publish(brake_ctrl_msg);
  vehicle_ctrl_pub_->publish(vehicle_ctrl_msg);
  park_ctrl_pub_->publish(park_ctrl_msg);
}
} // namespace control_converter
} // namespace pix_driver
