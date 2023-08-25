#ifndef PIX_ROBOBUS_DRIVER__REPORT_PARSER_HPP_
#define PIX_ROBOBUS_DRIVER__REPORT_PARSER_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

#include <can_msgs/msg/frame.hpp>


// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include <pix_robobus_driver_msgs/msg/chassis_sta_to_mobileye.hpp>
#include <pix_robobus_driver_msgs/msg/auto_ctrl_msg.hpp>
#include <pix_robobus_driver_msgs/msg/auto_remote_ctrl_msg.hpp>
#include <pix_robobus_driver_msgs/msg/throttle_report.hpp>
#include <pix_robobus_driver_msgs/msg/brake_report.hpp>
#include <pix_robobus_driver_msgs/msg/steering_report.hpp>
#include <pix_robobus_driver_msgs/msg/gear_report.hpp>
#include <pix_robobus_driver_msgs/msg/park_report.hpp>
#include <pix_robobus_driver_msgs/msg/vcu_report.hpp>
#include <pix_robobus_driver_msgs/msg/wheel_speed_report.hpp>
#include <pix_robobus_driver_msgs/msg/bms_report.hpp>
#include <pix_robobus_driver_msgs/msg/vehicle_door_report.hpp>
#include <pix_robobus_driver_msgs/msg/vehicle_mileage_fb.hpp>



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include <pix_robobus_driver/chassis_sta_to_mobileye.hpp>
#include <pix_robobus_driver/auto_ctrl_msg.hpp>
#include <pix_robobus_driver/auto_remote_ctrl_msg.hpp>
#include <pix_robobus_driver/throttle_report.hpp>
#include <pix_robobus_driver/brake_report.hpp>
#include <pix_robobus_driver/steering_report.hpp>
#include <pix_robobus_driver/gear_report.hpp>
#include <pix_robobus_driver/park_report.hpp>
#include <pix_robobus_driver/vcu_report.hpp>
#include <pix_robobus_driver/wheel_speed_report.hpp>
#include <pix_robobus_driver/bms_report.hpp>
#include <pix_robobus_driver/vehicle_door_report.hpp>
#include <pix_robobus_driver/vehicle_mileage_fb.hpp>
#include <pix_robobus_driver/vcu_pad_transfer.hpp>


namespace pix_robobus_driver
{
namespace report_parser
{

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
  // parameters of node
  Param param_;

  // is publish subscrber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_publish_sub_;
  bool is_publish_;

  // subscribers from socketcan interface
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;

  // publishers
  /** example
  rclcpp::Publisher<V2aBrakeStaFb>::SharedPtr brake_sta_fb_pub_;
  **/
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::ChassisStaToMobileye>::SharedPtr chassis_sta_to_mobileye_pub_;
  // rclcpp::Publisher<pix_robobus_driver_msgs::msg::AutoCtrlMsg>::SharedPtr auto_ctrl_msg_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg>::SharedPtr auto_remote_ctrl_msg_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::ThrottleReport>::SharedPtr throttle_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::BrakeReport>::SharedPtr brake_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::GearReport>::SharedPtr gear_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::ParkReport>::SharedPtr park_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::VcuReport>::SharedPtr vcu_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::BmsReport>::SharedPtr bms_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::VehicleDoorReport>::SharedPtr vehicle_door_report_pub_;
  rclcpp::Publisher<pix_robobus_driver_msgs::msg::VehicleMileageFb>::SharedPtr vehicle_mileage_fb_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vcu_pad_transfer_pub_;

  // publish msgs
  /** example
  V2aBrakeStaFb::ConstSharedPtr brake_sta_fb_ptr_;
  **/
  pix_robobus_driver_msgs::msg::ChassisStaToMobileye::ConstSharedPtr chassis_sta_to_mobileye_ptr_;
  pix_robobus_driver_msgs::msg::AutoCtrlMsg::ConstSharedPtr auto_ctrl_msg_ptr_;
  pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg::ConstSharedPtr auto_remote_ctrl_msg_ptr_;
  pix_robobus_driver_msgs::msg::ThrottleReport::ConstSharedPtr throttle_report_ptr_;
  pix_robobus_driver_msgs::msg::BrakeReport::ConstSharedPtr brake_report_ptr_;
  pix_robobus_driver_msgs::msg::SteeringReport::ConstSharedPtr steering_report_ptr_;
  pix_robobus_driver_msgs::msg::GearReport::ConstSharedPtr gear_report_ptr_;
  pix_robobus_driver_msgs::msg::ParkReport::ConstSharedPtr park_report_ptr_;
  pix_robobus_driver_msgs::msg::VcuReport::ConstSharedPtr vcu_report_ptr_;
  pix_robobus_driver_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_report_ptr_;
  pix_robobus_driver_msgs::msg::BmsReport::ConstSharedPtr bms_report_ptr_;
  pix_robobus_driver_msgs::msg::VehicleDoorReport::ConstSharedPtr vehicle_door_report_ptr_;
  pix_robobus_driver_msgs::msg::VehicleMileageFb::ConstSharedPtr vehicle_mileage_fb_ptr_;
  std_msgs::msg::Bool::ConstSharedPtr vcu_pad_transfer_ptr_;

  // can frame entities
  /** example
  V2adrivestafb530  v2a_drivestafb_530_entity_;
  **/
  ChassisStaToMobileye chassis_sta_to_mobileye_entity_;
  AutoCtrlMsg auto_ctrl_msg_entity_;
  AutoRemoteCtrlMsg auto_remote_ctrl_msg_entity_;
  ThrottleReport throttle_report_entity_;
  BrakeReport brake_report_entity_;
  SteeringReport steering_report_entity_;
  GearReport gear_report_entity_;
  ParkReport park_report_entity_;
  VcuReport vcu_report_entity_;
  WheelSpeedReport wheel_speed_report_entity_;
  BmsReport bms_report_entity_;
  VehicleDoorReport vehicle_door_report_entity_;
  VehicleMileageFb vehicle_mileage_fb_entity_;
  VCUPadTransfer vcu_pad_transfer_entity_;

  // msg reveived time
  /** example
  rclcpp::Time brake_sta_fb_received_time_;
  **/
  rclcpp::Time chassis_sta_to_mobileye_received_time_;
  rclcpp::Time auto_ctrl_msg_received_time_;
  rclcpp::Time auto_remote_ctrl_msg_received_time_;
  rclcpp::Time throttle_report_received_time_;
  rclcpp::Time brake_report_received_time_;
  rclcpp::Time steering_report_received_time_;
  rclcpp::Time gear_report_received_time_;
  rclcpp::Time park_report_received_time_;
  rclcpp::Time vcu_report_received_time_;
  rclcpp::Time wheel_speed_report_received_time_;
  rclcpp::Time bms_report_received_time_;
  rclcpp::Time vehicle_door_report_received_time_;
  rclcpp::Time vehicle_mileage_fb_received_time_;
  rclcpp::Time vcu_pad_transfer_received_time_;



  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ReportParser();
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
   * @brief parser can frames, convert can frames to pix_driver_msgs
   * 
   */
  void timerCallback();
};
} // report_parser
} // pix_robobus_driver
#endif // PIX_ROBOBUS_DRIVER__REPORT_PARSER_HPP_