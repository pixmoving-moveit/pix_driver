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
#include <rclcpp/rclcpp.hpp>

// geometry_msgs
#include <geometry_msgs/msg/twist_with_covariance.hpp>

// autoware_msgs
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

// tier4_msgs
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_api_msgs/msg/door_status.hpp>

// pix_sweeping_driver_msgs
#include <pix_sweeping_driver_msgs/msg/throttle_report.hpp>
#include <pix_sweeping_driver_msgs/msg/gear_report.hpp>
#include <pix_sweeping_driver_msgs/msg/brake_report.hpp>
#include <pix_sweeping_driver_msgs/msg/steering_report.hpp>
#include <pix_sweeping_driver_msgs/msg/vcu_report.hpp>

#include <pix_sweeping_driver_msgs/msg/wheel_speed_report.hpp>
#include <pix_sweeping_driver_msgs/msg/bms_report.hpp>


namespace pix_sweeping_driver
{
namespace report_converter
{
using ThrottleReport =  pix_sweeping_driver_msgs::msg::ThrottleReport;
using GearReport =  pix_sweeping_driver_msgs::msg::GearReport;
using BrakeReport =  pix_sweeping_driver_msgs::msg::BrakeReport;
using SteeringReport =  pix_sweeping_driver_msgs::msg::SteeringReport;
using VcuReport =  pix_sweeping_driver_msgs::msg::VcuReport;
using WheelSpeedReport =  pix_sweeping_driver_msgs::msg::WheelSpeedReport;

/**
 * @brief param structure of report converter node
 * @param period loop rate of publishers in hz
 * @param max_steering_angle max steering angle in radians
 * @param report_msg_timeout_ms timeout threshold of report msg from pix sweeping driver in ms
 * @param base_frame_id frame id of vehicle
 * @param steering_factor the factor that convert steering feedback signal value to autoware steering value
 */
struct Param
{
  double loop_rate;           // hz
  double max_steering_angle;  // radians
  int report_msg_timeout_ms;  // ms
  std::string base_frame_id;  // vehicle frame id
  double steering_factor;
};

class ReportConverter : public rclcpp::Node
{
private:
  Param param_;
  double steering_factor_;

  // publishers to Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>::SharedPtr steering_wheel_status_pub_;
  rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr door_status_pub_;
  
  // subscribers from pix sweeping driver
  rclcpp::Subscription<pix_sweeping_driver_msgs::msg::SteeringReport>::SharedPtr steer_report_sub_;
  rclcpp::Subscription<pix_sweeping_driver_msgs::msg::BrakeReport>::SharedPtr brake_report_sub_;
  rclcpp::Subscription<pix_sweeping_driver_msgs::msg::ThrottleReport>::SharedPtr throttle_report_sub_;
  rclcpp::Subscription<pix_sweeping_driver_msgs::msg::VcuReport>::SharedPtr vcu_report_sub_;
  rclcpp::Subscription<pix_sweeping_driver_msgs::msg::GearReport>::SharedPtr gear_report_sub_;
  rclcpp::Subscription<pix_sweeping_driver_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_report_sub_;

  // timers
  rclcpp::TimerBase::ConstSharedPtr timer_;

  // msg shared ptrs
  pix_sweeping_driver_msgs::msg::SteeringReport::ConstSharedPtr steer_report_ptr_;
  pix_sweeping_driver_msgs::msg::BrakeReport::ConstSharedPtr brake_report_ptr_;
  pix_sweeping_driver_msgs::msg::ThrottleReport::ConstSharedPtr throttle_report_ptr_;
  pix_sweeping_driver_msgs::msg::VcuReport::ConstSharedPtr vcu_report_ptr_;
  pix_sweeping_driver_msgs::msg::GearReport::ConstSharedPtr gear_report_ptr_; 
  pix_sweeping_driver_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_report_ptr_; 

  // msg received timestamps
  rclcpp::Time steer_report_received_timestamp_;
  rclcpp::Time brake_report_received_timestamp_;
  rclcpp::Time throttle_report_received_timestamp_;
  rclcpp::Time vcu_received_timestamp_;
  rclcpp::Time gear_report_received_timestamp_;
  rclcpp::Time wheel_speed_report_received_timestamp_;

  public :
  /**
   * @brief Construct a new Report Converter object
   *
   */
  ReportConverter();
  /**
   * @brief callback function that gets the steer status feedback from pix sweeping driver
   * 
   * @param msg 
   */
  void steerReportCallback(const pix_sweeping_driver_msgs::msg::SteeringReport::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the brake status feedback from pix sweeping driver
   * 
   * @param msg 
   */
  void brakeReportCallback(const pix_sweeping_driver_msgs::msg::BrakeReport::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the drive status feedback from pix sweeping driver
   * 
   * @param msg 
   */
  void throttleReportCallback(const pix_sweeping_driver_msgs::msg::ThrottleReport::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the vehicle status feedback from pix sweeping driver
   * 
   * @param msg 
   */
  void vcuReportCallback(
    const pix_sweeping_driver_msgs::msg::VcuReport::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the Gear status feedback from pix sweeping driver
   * 
   * @param msg 
   */
  void gearReportCallback(
    const pix_sweeping_driver_msgs::msg::GearReport::ConstSharedPtr & msg);
  
  /**
   * @brief callback function that gets the  WheelSpeedReport feedback from pix sweeping driver
   * 
   * @param msg 
   */
  void wheelSpeedReportCallback(
    const pix_sweeping_driver_msgs::msg::WheelSpeedReport::ConstSharedPtr & msg);
  
  void timerCallback();
};
} // report_converter
} // pix_sweeping_driver