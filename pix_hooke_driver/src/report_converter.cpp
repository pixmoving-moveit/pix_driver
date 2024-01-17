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

#include <pix_hooke_driver/report_converter.hpp>

namespace pix_hooke_driver
{
namespace report_converter
{
ReportConverter::ReportConverter() : rclcpp::Node("report_converter")
{
  // initialize node parameters
  param_.report_msg_timeout_ms =
    declare_parameter("report_msg_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);
  param_.max_steering_angle = declare_parameter("max_steering_angle", 0.5236);
  param_.steering_factor = param_.max_steering_angle / 500.0;
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");

  // initialize msg received timestamps
  steer_sta_fb_received_timestamp_ = this->now();
  brake_sta_fb_received_timestamp_ = this->now();
  drive_sta_fb_received_timestamp_ = this->now();
  vehicle_sta_fb_received_timestamp_ = this->now();
  vehicle_work_sta_fb_received_timestamp_ = this->now();

  // initialize subscribers
  brake_sta_fb_sub_ = create_subscription<pix_hooke_driver_msgs::msg::V2aBrakeStaFb>(
    "/pix_hooke/v2a_brakestafb", 1,
    std::bind(&ReportConverter::brakeStaFbCallback, this, std::placeholders::_1));
  drive_sta_fb_sub_ = create_subscription<pix_hooke_driver_msgs::msg::V2aDriveStaFb>(
    "/pix_hooke/v2a_drivestafb", 1,
    std::bind(&ReportConverter::driveStaFbCallback, this, std::placeholders::_1));
  steer_sta_fb_sub_ = create_subscription<pix_hooke_driver_msgs::msg::V2aSteerStaFb>(
    "/pix_hooke/v2a_steerstafb", 1,
    std::bind(&ReportConverter::steerStaFbCallback, this, std::placeholders::_1));
  vehicle_sta_fb_sub_ = create_subscription<pix_hooke_driver_msgs::msg::V2aVehicleStaFb>(
    "/pix_hooke/v2a_vehiclestafb", 1,
    std::bind(&ReportConverter::vehicleStaFbCallback, this, std::placeholders::_1));
  vehicle_work_sta_fb_sub_ = create_subscription<pix_hooke_driver_msgs::msg::V2aVehicleWorkStaFb>(
    "/pix_hooke/v2a_vehicleworkstafb", 1,
    std::bind(&ReportConverter::vehicleWorkStaFbCallback, this, std::placeholders::_1));

  // initialize publishers
  control_mode_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  turn_indicators_status_pub_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  hazard_lights_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  actuation_status_pub_ = create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "/vehicle/status/actuation_status", 1);
  steering_wheel_status_pub_ =
    create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>(
      "/vehicle/status/steering_wheel_status", 1);
  door_status_pub_ =
    create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);
  
  // initialize of timer
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
    std::bind(&ReportConverter::timerCallback, this));
}

void ReportConverter::steerStaFbCallback(const pix_hooke_driver_msgs::msg::V2aSteerStaFb::ConstSharedPtr & msg)
{
  steer_sta_fb_received_timestamp_ = this->now();
  steer_sta_fb_ptr_ = msg;
}

void ReportConverter::brakeStaFbCallback(const pix_hooke_driver_msgs::msg::V2aBrakeStaFb::ConstSharedPtr & msg)
{
  brake_sta_fb_received_timestamp_ = this->now();
  brake_sta_fb_ptr_ = msg;
}

void ReportConverter::driveStaFbCallback(const pix_hooke_driver_msgs::msg::V2aDriveStaFb::ConstSharedPtr & msg)
{
  drive_sta_fb_received_timestamp_ = this->now();
  drive_sta_fb_ptr_ = msg;
}

void ReportConverter::vehicleStaFbCallback(const pix_hooke_driver_msgs::msg::V2aVehicleStaFb::ConstSharedPtr & msg)
{
  vehicle_sta_fb_received_timestamp_ = this->now();
  vehicle_sta_fb_ptr_ = msg;
}

void ReportConverter::vehicleWorkStaFbCallback(const pix_hooke_driver_msgs::msg::V2aVehicleWorkStaFb::ConstSharedPtr & msg)
{
  vehicle_work_sta_fb_received_timestamp_ = this->now();
  vehicle_work_sta_fb_ptr_ = msg;
}

void ReportConverter::timerCallback()
{
  const rclcpp::Time current_time = this->now();
  const double steer_sta_fb_delta_time_ms =
    (current_time - steer_sta_fb_received_timestamp_).seconds() * 1000.0;
  const double brake_sta_fb_delta_time_ms =
    (current_time - brake_sta_fb_received_timestamp_).seconds() * 1000.0;
  const double drive_sta_fb_delta_time_ms =
    (current_time - drive_sta_fb_received_timestamp_).seconds() * 1000.0;
  const double vehicle_sta_fb_delta_time_ms =
    (current_time - vehicle_sta_fb_received_timestamp_).seconds() * 1000.0;
  const double vehicle_work_sta_fb_delta_time_ms =
    (current_time - vehicle_work_sta_fb_received_timestamp_).seconds() * 1000.0;

  if (steer_sta_fb_ptr_ == nullptr || brake_sta_fb_ptr_ == nullptr || drive_sta_fb_ptr_ == nullptr ||
      vehicle_sta_fb_ptr_ == nullptr || steer_sta_fb_delta_time_ms>param_.report_msg_timeout_ms ||
      brake_sta_fb_delta_time_ms>param_.report_msg_timeout_ms || drive_sta_fb_delta_time_ms>param_.report_msg_timeout_ms ||
      vehicle_sta_fb_delta_time_ms>param_.report_msg_timeout_ms)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(), "vital msgs not received or timeout");
    return;
  }

  if(vehicle_work_sta_fb_ptr_ == nullptr || vehicle_work_sta_fb_delta_time_ms>param_.report_msg_timeout_ms)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(), "vehicle work sta fb not received or timeout");
  }

  // autoware msgs
  autoware_auto_vehicle_msgs::msg::GearReport gear_msg;
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_report_msg;
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_lights_report_msg;
  autoware_auto_vehicle_msgs::msg::SteeringReport steer_report_msg;
  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_report_msg;
  // tier4 msgs
  tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_status_stamped_msg;
  tier4_vehicle_msgs::msg::SteeringWheelStatusStamped steering_wheel_status_msg;
  tier4_api_msgs::msg::DoorStatus door_status_msg;

  // making gear msg
  gear_msg.stamp = current_time;
  switch (drive_sta_fb_ptr_->vcu_chassis_gear_fb) {
    case static_cast<int8_t>(VCU_CHASSISGEARFB_D):
      gear_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
      break;
    case static_cast<int8_t>(VCU_CHASSISGEARFB_N):
      gear_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL;
      break;
    case static_cast<int8_t>(VCU_CHASSISGEARFB_NO_USE):
      gear_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NONE;
      break;
    case static_cast<int8_t>(VCU_CHASSISGEARFB_R):
      gear_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::REVERSE;
      break;
    default:
      gear_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NONE;
      break;
  }
  gear_status_pub_->publish(gear_msg);

  // making velocity
  velocity_report_msg.header.frame_id = param_.base_frame_id;
  velocity_report_msg.header.stamp = current_time;
  velocity_report_msg.longitudinal_velocity = drive_sta_fb_ptr_->vcu_chassis_speed_fb;
  vehicle_twist_pub_->publish(velocity_report_msg);

  // make steering angle
  steer_report_msg.stamp = current_time;
  steer_report_msg.steering_tire_angle = -1.0 * steer_sta_fb_ptr_->vcu_chassis_steer_angle_fb * param_.steering_factor;
  steering_status_pub_->publish(steer_report_msg);

  // make control mode
  control_mode_report_msg.stamp = current_time;
  switch (vehicle_work_sta_fb_ptr_->vcu_driving_mode_fb)
  {
  case static_cast<int8_t>(VCU_DRIVINGMODEFB_SELF_DRIVING):
    control_mode_report_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    break;
  case static_cast<int8_t>(VCU_DRIVINGMODEFB_STANDBY):
    control_mode_report_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::DISENGAGED;
    break;
  case static_cast<int8_t>(VCU_DRIVINGMODEFB_REMOTE):
    control_mode_report_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
    break;
  default:
    control_mode_report_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::NOT_READY;
    break;
  }
  control_mode_pub_->publish(control_mode_report_msg);

  // hazard lights status
  hazard_lights_report_msg.stamp = current_time;
  if(vehicle_sta_fb_ptr_->vcu_vehicle_hazard_war_lamp_fb ==VCU_VEHICLEHAZARDWARLAMPFB_ON)
  {
    hazard_lights_report_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE;
  }else{
    hazard_lights_report_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE;
  }

  // turn indicators, pix chassi feedbacks LEFT light and RIGHT light separately, if the hazard light blink, it will output ENABLE_LEFT as default
  turn_indicators_report_msg.stamp = current_time;
  if(vehicle_sta_fb_ptr_->vcu_vehicle_left_lamp_fb==VCU_VEHICLELEFTLAMPFB_OFF && vehicle_sta_fb_ptr_->vcu_vehicle_right_lamp_fb==VCU_VEHICLERIGHTLAMPFB_OFF)
  {
    turn_indicators_report_msg.report =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  } else if (vehicle_sta_fb_ptr_->vcu_vehicle_right_lamp_fb == VCU_VEHICLERIGHTLAMPFB_ON) {
    turn_indicators_report_msg.report =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
  } else if(vehicle_sta_fb_ptr_->vcu_vehicle_left_lamp_fb==VCU_VEHICLELEFTLAMPFB_ON) {
    turn_indicators_report_msg.report =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
  }
  turn_indicators_status_pub_->publish(turn_indicators_report_msg);

  // tier4 vehicle msgs, acutation msgs
  actuation_status_stamped_msg.header.stamp = current_time;
  actuation_status_stamped_msg.header.frame_id = "base_link";
  actuation_status_stamped_msg.status.accel_status =
    drive_sta_fb_ptr_->vcu_chassis_throttle_padl_fb / 100.0;
  actuation_status_stamped_msg.status.brake_status =
    brake_sta_fb_ptr_->vcu_chassis_brake_padl_fb / 100.0;
  // have no idea about the mean of steer_status in ActuationStatusStamped, so it should be empty
  actuation_status_pub_->publish(actuation_status_stamped_msg);
  // to be done, tier4 msgs
}

} // namespace report_converter
} // namespace pix_driver
