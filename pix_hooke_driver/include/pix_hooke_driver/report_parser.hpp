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

#ifndef PIX_HOOKE_DRIVER__REPORT_PARSER_HPP_
#define PIX_HOOKE_DRIVER__REPORT_PARSER_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

#include <can_msgs/msg/frame.hpp>

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

#include <pix_hooke_driver/v2a_drivestafb_530.hpp>
#include <pix_hooke_driver/v2a_brakestafb_531.hpp>
#include <pix_hooke_driver/v2a_steerstafb_532.hpp>
#include <pix_hooke_driver/v2a_vehicleworkstafb_534.hpp>
#include <pix_hooke_driver/v2a_powerstafb_535.hpp>
#include <pix_hooke_driver/v2a_vehiclestafb_536.hpp>
#include <pix_hooke_driver/v2a_vehiclefltsta_537.hpp>
#include <pix_hooke_driver/v2a_chassiswheelrpmfb_539.hpp>
#include <pix_hooke_driver/v2a_chassiswheeltirepressfb_540.hpp>
#include <pix_hooke_driver/v2a_chassiswheelanglefb_541.hpp>


namespace pix_hooke_driver
{
namespace report_parser
{
using V2aBrakeStaFb = pix_hooke_driver_msgs::msg::V2aBrakeStaFb;
using V2aChassisWheelAngleFb = pix_hooke_driver_msgs::msg::V2aChassisWheelAngleFb;
using V2aChassisWheelRpmFb = pix_hooke_driver_msgs::msg::V2aChassisWheelRpmFb;
using V2aChassisWheelTirePressFb = pix_hooke_driver_msgs::msg::V2aChassisWheelTirePressFb;
using V2aDriveStaFb = pix_hooke_driver_msgs::msg::V2aDriveStaFb;
using V2aPowerStaFb = pix_hooke_driver_msgs::msg::V2aPowerStaFb;
using V2aSteerStaFb = pix_hooke_driver_msgs::msg::V2aSteerStaFb;
using V2aVehicleFltSta = pix_hooke_driver_msgs::msg::V2aVehicleFltSta;
using V2aVehicleStaFb = pix_hooke_driver_msgs::msg::V2aVehicleStaFb;
using V2aVehicleWorkStaFb = pix_hooke_driver_msgs::msg::V2aVehicleWorkStaFb;

/**
 * @brief param structure of report parser node
 * @param base_frame_id frame id of vehicle
 * @param loop_rate loop rate of publishers in hz
 * @param report_timeout_ms timeout threshold of report can Frame msg from canbus driver
 */
struct Param
{
  std::string base_frame_id;
  double loop_rate;
  int report_timeout_ms;
};

class ReportParser : public rclcpp::Node
{
private:
  // params
  Param param_;

  // is publish subscrber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_publish_sub_;
  bool is_publish_;

  // subscribers from socketcan interface
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;

  // publisher to pix driver autoware interface
  rclcpp::Publisher<V2aBrakeStaFb>::SharedPtr brake_sta_fb_pub_;
  rclcpp::Publisher<V2aChassisWheelAngleFb>::SharedPtr chassis_wheel_angle_fb_pub_;
  rclcpp::Publisher<V2aChassisWheelRpmFb>::SharedPtr chassis_wheel_rpm_fb_pub_;
  rclcpp::Publisher<V2aChassisWheelTirePressFb>::SharedPtr chassis_wheel_tire_press_fb_pub_;
  rclcpp::Publisher<V2aDriveStaFb>::SharedPtr drive_sta_fb_pub_;
  rclcpp::Publisher<V2aPowerStaFb>::SharedPtr power_sta_fb_pub_;
  rclcpp::Publisher<V2aSteerStaFb>::SharedPtr steer_sta_fb_pub_;
  rclcpp::Publisher<V2aVehicleFltSta>::SharedPtr vehicle_flt_sta_pub_;
  rclcpp::Publisher<V2aVehicleStaFb>::SharedPtr vehicle_sta_fb_pub_;
  rclcpp::Publisher<V2aVehicleWorkStaFb>::SharedPtr vehicle_work_sta_fb_pub_;

  // publish msgs
  V2aBrakeStaFb::ConstSharedPtr brake_sta_fb_ptr_;
  V2aChassisWheelAngleFb::ConstSharedPtr chassis_wheel_angle_fb_ptr_;
  V2aChassisWheelRpmFb::ConstSharedPtr chassis_wheel_rpm_fb_ptr_;
  V2aChassisWheelTirePressFb::ConstSharedPtr chassis_wheel_tire_press_fb_ptr_;
  V2aDriveStaFb::ConstSharedPtr drive_sta_fb_ptr_;
  V2aPowerStaFb::ConstSharedPtr power_sta_fb_ptr_;
  V2aSteerStaFb::ConstSharedPtr steer_sta_fb_ptr_;
  V2aVehicleFltSta::ConstSharedPtr vehicle_flt_sta_ptr_;
  V2aVehicleStaFb::ConstSharedPtr vehicle_sta_fb_ptr_;
  V2aVehicleWorkStaFb::ConstSharedPtr vehicle_work_sta_fb_ptr_;

  V2adrivestafb530  v2a_drivestafb_530_entity_;
  V2abrakestafb531  v2a_brakestafb_531_entity_;
  V2asteerstafb532  v2a_steerstafb_532_entity_;
  V2avehicleworkstafb534  v2a_vehicleworkstafb_534_entity_;
  V2apowerstafb535  v2a_powerstafb_535_entity_;
  V2avehiclestafb536  v2a_vehiclestafb_536_entity_;
  V2avehiclefltsta537  v2a_vehiclefltsta_537_entity_;
  V2achassiswheelrpmfb539  v2a_chassiswheelrpmfb_539_entity_;
  V2achassiswheeltirepressfb540  v2a_chassiswheeltirepressfb_540_entity_;
  V2achassiswheelanglefb541  v2a_chassiswheelanglefb_541_entity_;

  // brake_sta_fb
  // chassis_wheel_angle_fb
  // chassis_wheel_rpm_fb
  // chassis_wheel_tire_press_fb
  // drive_sta_fb
  // power_sta_fb
  // steer_sta_fb
  // vehicle_flt_sta
  // vehicle_sta_fb
  // vehicle_work_sta_fb

  // msg received time
  rclcpp::Time brake_sta_fb_received_time_;
  rclcpp::Time chassis_wheel_angle_fb_received_time_;
  rclcpp::Time chassis_wheel_rpm_fb_received_time_;
  rclcpp::Time chassis_wheel_tire_press_fb_received_time_;
  rclcpp::Time drive_sta_fb_received_time_;
  rclcpp::Time power_sta_fb_received_time_;
  rclcpp::Time steer_sta_fb_received_time_;
  rclcpp::Time vehicle_flt_sta_received_time_;
  rclcpp::Time vehicle_sta_fb_received_time_;
  rclcpp::Time vehicle_work_sta_fb_received_time_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;
public:
  ReportParser(/* args */);

  // callback
  /**
   * @brief callback function of can Frame msgs, to store the data to member variable
   * 
   * @param msg 
   */
  void callbackCan(const can_msgs::msg::Frame::ConstSharedPtr & msg);
  /**
   * @brief callback function of Bool msg, to store the data to member variable, decide publish report msgs or not
   * 
   * @param msg 
   */
  void callbackIsPublish(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  /**
   * @brief parser can frames, convert can frames to pix_hooke_driver_msgs
   * 
   */
  void timerCallback();
};
} // namespace report_parser
} // namespace pix_hooke_driver

#endif // PIX_HOOKE_DRIVER__REPORT_PARSER_HPP_