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
#include <geometry_msgs/msg/vector3_stamped.hpp>

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

// pix_hooke_driver_msgs
#include <pix_hooke_driver_msgs/msg/v2a_brake_sta_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_chassis_wheel_angle_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_chassis_wheel_rpm_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_chassis_wheel_tire_press_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_drive_sta_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_power_sta_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_steer_sta_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_vehicle_flt_sta.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_vehicle_sta_fb.hpp>
#include <pix_hooke_driver_msgs/msg/v2a_vehicle_work_sta_fb.hpp>

namespace pix_hooke_driver
{
namespace report_converter
{
enum { VCU_CHASSISGEARFB_NO_USE, VCU_CHASSISGEARFB_D, VCU_CHASSISGEARFB_N, VCU_CHASSISGEARFB_R };
enum { VCU_CHASSISDRIVERENSTA_DISABLE, VCU_CHASSISDRIVERENSTA_ENABLE };
enum {
  VCU_DRIVINGMODEFB_STANDBY,
  VCU_DRIVINGMODEFB_SELF_DRIVING,
  VCU_DRIVINGMODEFB_REMOTE,
  VCU_DRIVINGMODEFB_MAN
};
enum { VCU_VEHICLEHAZARDWARLAMPFB_OFF, VCU_VEHICLEHAZARDWARLAMPFB_ON };
enum { VCU_VEHICLELEFTLAMPFB_OFF, VCU_VEHICLELEFTLAMPFB_ON };
enum { VCU_VEHICLERIGHTLAMPFB_OFF, VCU_VEHICLERIGHTLAMPFB_ON };
enum {
  VCU_CHASSISSTEERMODEFB_FRONT_ACKERMAN,
  VCU_CHASSISSTEERMODEFB_SAME_FRONT_AND_BACK,
  VCU_CHASSISSTEERMODEFB_FRONT_DIFFERENT_BACK,
  VCU_CHASSISSTEERMODEFB_BACK_ACKRMAN,
  VCU_CHASSISSTEERMODEFB_FRONT_BACK
};

/**
 * @brief param structure of report converter node
 * @param period loop rate of publishers in hz
 * @param max_steering_angle max steering angle in radians
 * @param report_msg_timeout_ms timeout threshold of report msg from pix hooke driver in ms
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
  bool use_steer_mode_correct;
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
   rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr slip_angle_pub_;
  
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>::SharedPtr steering_wheel_status_pub_;
  rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr door_status_pub_;
  
  // subscribers from pix hooke driver
  rclcpp::Subscription<pix_hooke_driver_msgs::msg::V2aSteerStaFb>::SharedPtr steer_sta_fb_sub_;
  rclcpp::Subscription<pix_hooke_driver_msgs::msg::V2aBrakeStaFb>::SharedPtr brake_sta_fb_sub_;
  rclcpp::Subscription<pix_hooke_driver_msgs::msg::V2aDriveStaFb>::SharedPtr drive_sta_fb_sub_;
  rclcpp::Subscription<pix_hooke_driver_msgs::msg::V2aVehicleStaFb>::SharedPtr vehicle_sta_fb_sub_;
  rclcpp::Subscription<pix_hooke_driver_msgs::msg::V2aVehicleWorkStaFb>::SharedPtr vehicle_work_sta_fb_sub_;

  // timers
  rclcpp::TimerBase::ConstSharedPtr timer_;

  // msg shared ptrs
  pix_hooke_driver_msgs::msg::V2aSteerStaFb::ConstSharedPtr steer_sta_fb_ptr_;
  pix_hooke_driver_msgs::msg::V2aBrakeStaFb::ConstSharedPtr brake_sta_fb_ptr_;
  pix_hooke_driver_msgs::msg::V2aDriveStaFb::ConstSharedPtr drive_sta_fb_ptr_;
  pix_hooke_driver_msgs::msg::V2aVehicleStaFb::ConstSharedPtr vehicle_sta_fb_ptr_;
  pix_hooke_driver_msgs::msg::V2aVehicleWorkStaFb::ConstSharedPtr vehicle_work_sta_fb_ptr_;

  // msg received timestamps
  rclcpp::Time steer_sta_fb_received_timestamp_;
  rclcpp::Time brake_sta_fb_received_timestamp_;
  rclcpp::Time drive_sta_fb_received_timestamp_;
  rclcpp::Time vehicle_sta_fb_received_timestamp_;
  rclcpp::Time vehicle_work_sta_fb_received_timestamp_;

  public :
  /**
   * @brief Construct a new Report Converter object
   *
   */
  ReportConverter();
  /**
   * @brief callback function that gets the steer status feedback from pix hooke driver
   * 
   * @param msg 
   */
  void steerStaFbCallback(const pix_hooke_driver_msgs::msg::V2aSteerStaFb::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the brake status feedback from pix hooke driver
   * 
   * @param msg 
   */
  void brakeStaFbCallback(const pix_hooke_driver_msgs::msg::V2aBrakeStaFb::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the drive status feedback from pix hooke driver
   * 
   * @param msg 
   */
  void driveStaFbCallback(const pix_hooke_driver_msgs::msg::V2aDriveStaFb::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the vehicle status feedback from pix hooke driver
   * 
   * @param msg 
   */
  void vehicleStaFbCallback(
    const pix_hooke_driver_msgs::msg::V2aVehicleStaFb::ConstSharedPtr & msg);
  /**
   * @brief callback function that gets the vehicle work status feedback from pix hooke driver
   * 
   * @param msg 
   */
  void vehicleWorkStaFbCallback(
    const pix_hooke_driver_msgs::msg::V2aVehicleWorkStaFb::ConstSharedPtr & msg);
  void timerCallback();
};
} // report_converter
} // pix_hooke_driver