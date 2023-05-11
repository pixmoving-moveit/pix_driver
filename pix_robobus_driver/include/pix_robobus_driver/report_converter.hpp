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

// pix_robobus_driver_msgs
#include <pix_robobus_driver_msgs/msg/throttle_report.hpp>
#include <pix_robobus_driver_msgs/msg/brake_report.hpp>
#include <pix_robobus_driver_msgs/msg/steering_report.hpp>
#include <pix_robobus_driver_msgs/msg/gear_report.hpp>
#include <pix_robobus_driver_msgs/msg/vcu_report.hpp>
#include <pix_robobus_driver_msgs/msg/vehicle_door_report.hpp>


namespace pix_robobus_driver
{
namespace report_converter
{
enum { GEAR_INVALID, GEAR_PARK, GEAR_REVERSE, GEAR_NEUTRAL, GEAR_DRIVE};
enum { Disable, Enable };
enum {
  VEHICLE_Manual_Remote_Mode,
  VEHICLE_Auto_Mode,
  VEHICLE_Emergency_Mode,
  VEHICLE_Standby_Mode
};
enum { 
  TurnLight_Turnlampsts_OFF, 
  TurnLight_Left_Turnlampsts_ON,
  TurnLight_Right_Turnlampsts_ON,
  TurnLight_Hazard_Warning_Lampsts_ON
};
enum { DOOR_OPENING, DOOR_NONE };


/**
 * @brief param structure of report converter node
 * @param period loop rate of publishers in hz
 * @param max_steering_angle max steering angle in radians
 * @param report_msg_timeout_ms timeout threshold of report msg from pix robobus driver in ms
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
  
  // subscribers from pix robobus driver
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::ThrottleReport>::SharedPtr throttle_report_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::BrakeReport>::SharedPtr brake_report_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::SteeringReport>::SharedPtr steering_report_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::GearReport>::SharedPtr gear_report_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::VcuReport>::SharedPtr vcu_report_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::VehicleDoorReport>::SharedPtr vehicle_door_report_sub_;

  // timers
  rclcpp::TimerBase::ConstSharedPtr timer_;

  // msg shared ptrs
  pix_robobus_driver_msgs::msg::ThrottleReport::ConstSharedPtr throttle_report_ptr_;
  pix_robobus_driver_msgs::msg::BrakeReport::ConstSharedPtr brake_report_ptr_;
  pix_robobus_driver_msgs::msg::SteeringReport::ConstSharedPtr steering_report_ptr_;
  pix_robobus_driver_msgs::msg::GearReport::ConstSharedPtr gear_report_ptr_;
  pix_robobus_driver_msgs::msg::VcuReport::ConstSharedPtr vcu_report_ptr_;
  pix_robobus_driver_msgs::msg::VehicleDoorReport::ConstSharedPtr vehicle_door_report_ptr_;

  // msg received timestamps
  rclcpp::Time throttle_report_received_timestamp_;
  rclcpp::Time brake_report_received_timestamp_;
  rclcpp::Time steering_report_received_timestamp_;
  rclcpp::Time gear_report_received_timestamp_;
  rclcpp::Time vcu_report_received_timestamp_;
  rclcpp::Time vehicle_door_report_received_timestamp_;

  // autoware msgs
  autoware_auto_vehicle_msgs::msg::GearReport gear_msg_;
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_report_msg_;
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_lights_report_msg_;       
  autoware_auto_vehicle_msgs::msg::SteeringReport steer_report_msg_;                   
  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg_;                
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_report_msg_;   
  // tier4 msgs
  tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_status_stamped_msg_;
  tier4_vehicle_msgs::msg::SteeringWheelStatusStamped steering_wheel_status_msg_;
  tier4_api_msgs::msg::DoorStatus door_status_msg;

  public:
  /**
   * @brief Construct a new Report Converter object
   *
   */
  ReportConverter();
  void timerCallback();

  /**
   * @brief callback function that gets the drive status feedback from pix robobus driver
   * @param msg 
   */
  void throttleReportCallback(const pix_robobus_driver_msgs::msg::ThrottleReport::ConstSharedPtr & msg);

  /**
   * @brief callback function that gets the brake status feedback from pix robobus driver
   * @param msg 
   */
  void brakeReportCallback(const pix_robobus_driver_msgs::msg::BrakeReport::ConstSharedPtr & msg);

  /**
   * @brief callback function that gets the steer status feedback from pix ron o bu s driver
   * @param msg 
   */
  void steeringReportCallback(const pix_robobus_driver_msgs::msg::SteeringReport::ConstSharedPtr & msg);

  /**
   * @brief callback function that gets the  gear status feedback from pix robobus driver
   * @param msg 
   */
  void gearReportCallback(const pix_robobus_driver_msgs::msg::GearReport::ConstSharedPtr & msg);

  /**
   * @brief callback function that gets the vehicle status feedback from pix robobus driver
   * @param msg 
   */
  void vcuReportCallback(const pix_robobus_driver_msgs::msg::VcuReport::ConstSharedPtr & msg);

  /**
   * @brief callback function that gets the door status feedback from pix robobus driver
   * @param msg 
   */
  void vehicleDoorReportCallback(const pix_robobus_driver_msgs::msg::VehicleDoorReport::ConstSharedPtr & msg);
  
};
} // report_converter
} // pix_robobus_driver