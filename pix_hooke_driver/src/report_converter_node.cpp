/**
 * @file report_converter_node.cpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief convert pix_hooke_driver_msgs to autoware_vehicle_msgs
 * @version 0.1
 * @date 2022-12-09
 * 
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 * 
 */

#include <memory>
#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32.h"

#include "pix_hooke_driver_msgs/v2a_drivestafb_530.h"
#include "pix_hooke_driver_msgs/v2a_steerstafb_532.h"
#include "pix_hooke_driver_msgs/v2a_vehiclestafb_536.h"
#include "pix_hooke_driver_msgs/v2a_vehicleworkstafb_534.h"

#include "autoware_vehicle_msgs/ControlMode.h"
#include "autoware_vehicle_msgs/ShiftStamped.h"
#include "autoware_vehicle_msgs/Steering.h"
#include "autoware_vehicle_msgs/TurnSignal.h"

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

struct Params
{
  double report_period;       // hz
  double max_steering_angle;  // radians
  std::string sub_steer_topic;
  std::string sub_drive_topic;
  std::string sub_vehicle_topic;
  std::string sub_vehicle_work_topic;

  std::string pub_twist_topic;
  std::string pub_shift_topic;
  std::string pub_steer_topic;
  std::string pub_turn_signal_topic;
  std::string pub_control_mode_topic;
  std::string pub_velocity_topic;
};

class ReportConverter
{
private:
  Params report_converter_params_;
  double steering_factor_;

public:
  ReportConverter(/* args */);
  ~ReportConverter();
  void steerCallback(const pix_hooke_driver_msgs::v2a_steerstafb_532ConstPtr & msg);
  void vehicleCallback(const pix_hooke_driver_msgs::v2a_vehiclestafb_536ConstPtr & msg);
  void driveCallback(const pix_hooke_driver_msgs::v2a_drivestafb_530ConstPtr & msg);
  void vehicleWorkCallback(const pix_hooke_driver_msgs::v2a_vehicleworkstafb_534ConstPtr & msg);
  void timer_callback(const ros::TimerEvent & te);

protected:
  // node handle
  ros::NodeHandle nh_;
  // publishers
  ros::Subscriber sub_steer_;
  ros::Subscriber sub_drive_;
  ros::Subscriber sub_vehicle_;
  ros::Subscriber sub_vehicle_work_;
  // subscribers
  ros::Publisher pub_steer_;
  ros::Publisher pub_shift_;
  ros::Publisher pub_turn_signal_;
  ros::Publisher pub_control_mode_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_velocity_;
  // timer
  ros::Timer timer_;
};

ReportConverter::ReportConverter()
{
  // params
  {
    nh_.getParam("/vehicle_info/max_steer_angle", report_converter_params_.max_steering_angle);
    nh_.getParam("report_converter_node/control_period", report_converter_params_.report_period);
    nh_.getParam("report_converter_node/sub_steer_topic", report_converter_params_.sub_steer_topic);
    nh_.getParam("report_converter_node/sub_drive_topic", report_converter_params_.sub_drive_topic);
    nh_.getParam(
      "report_converter_node/sub_vehicle_topic", report_converter_params_.sub_vehicle_topic);
    nh_.getParam(
      "report_converter_node/sub_vehicle_work_topic",
      report_converter_params_.sub_vehicle_work_topic);
    nh_.getParam("report_converter_node/pub_twist_topic", report_converter_params_.pub_twist_topic);
    nh_.getParam("report_converter_node/pub_shift_topic", report_converter_params_.pub_shift_topic);
    nh_.getParam("report_converter_node/pub_steer_topic", report_converter_params_.pub_steer_topic);
    nh_.getParam(
      "report_converter_node/pub_turn_signal_topic",
      report_converter_params_.pub_turn_signal_topic);
    nh_.getParam(
      "report_converter_node/pub_control_mode_topic",
      report_converter_params_.pub_control_mode_topic);
    nh_.getParam(
      "report_converter_node/pub_velocity_topic", report_converter_params_.pub_velocity_topic);

    steering_factor_ = report_converter_params_.max_steering_angle/360.0;
  }
  // subscribers
  sub_steer_ = nh_.subscribe(
    report_converter_params_.sub_steer_topic, 1, &ReportConverter::steerCallback, this);
  sub_drive_ = nh_.subscribe(
    report_converter_params_.sub_drive_topic, 1, &ReportConverter::driveCallback, this);
  sub_vehicle_ = nh_.subscribe(
    report_converter_params_.sub_vehicle_topic, 1, &ReportConverter::vehicleCallback, this);
  sub_vehicle_work_ = nh_.subscribe(
    report_converter_params_.sub_vehicle_work_topic, 1, &ReportConverter::vehicleWorkCallback,
    this);
  // publishers
  pub_steer_ = nh_.advertise<autoware_vehicle_msgs::Steering>(
    report_converter_params_.pub_steer_topic, 5, false);
  pub_shift_ = nh_.advertise<autoware_vehicle_msgs::ShiftStamped>(
    report_converter_params_.pub_shift_topic, 5, false);
  pub_turn_signal_ = nh_.advertise<autoware_vehicle_msgs::TurnSignal>(
    report_converter_params_.pub_turn_signal_topic, 5, false);
  pub_control_mode_ = nh_.advertise<autoware_vehicle_msgs::ControlMode>(
    report_converter_params_.pub_control_mode_topic, 5, false);
  pub_twist_ =
    nh_.advertise<geometry_msgs::TwistStamped>(report_converter_params_.pub_twist_topic, 5, false);
  pub_velocity_ =
    nh_.advertise<std_msgs::Float32>(report_converter_params_.pub_velocity_topic, 5, false);
  // timers
  // timer_ = nh_.createTimer(ros::Duration(1.0/report_converter_params_.report_period), &ReportConverter::timer_callback, this);
}

ReportConverter::~ReportConverter() {}

void ReportConverter::steerCallback(const pix_hooke_driver_msgs::v2a_steerstafb_532ConstPtr & msg)
{
  autoware_vehicle_msgs::Steering aw_steer_msg;
  aw_steer_msg.header.frame_id = "base_link";
  aw_steer_msg.header.stamp = msg->header.stamp;
  aw_steer_msg.data = -1.0 * msg->VCU_ChassisSteerAngleFb * steering_factor_;
  pub_steer_.publish(aw_steer_msg);
}

void ReportConverter::vehicleCallback(
  const pix_hooke_driver_msgs::v2a_vehiclestafb_536ConstPtr & msg)
{
  autoware_vehicle_msgs::TurnSignal turn_signal_msg;

  turn_signal_msg.header.frame_id = "base_link";
  turn_signal_msg.header.stamp = msg->header.stamp;
  if (msg->VCU_VehicleLeftLampFb == 1) {
    turn_signal_msg.data = autoware_vehicle_msgs::TurnSignal::LEFT;
  } else if (msg->VCU_VehicleRightLampFb == 1) {
    turn_signal_msg.data = autoware_vehicle_msgs::TurnSignal::RIGHT;
  } else if (msg->VCU_VehicleHazardWarLampFb == 1) {
    turn_signal_msg.data = autoware_vehicle_msgs::TurnSignal::HAZARD;
  } else {
    turn_signal_msg.data = autoware_vehicle_msgs::TurnSignal::NONE;
  }
  pub_turn_signal_.publish(turn_signal_msg);
}

void ReportConverter::driveCallback(const pix_hooke_driver_msgs::v2a_drivestafb_530ConstPtr & msg)
{
  autoware_vehicle_msgs::ShiftStamped shift_msg;
  shift_msg.header.frame_id = "base_link";
  shift_msg.header.stamp = msg->header.stamp;
  switch (msg->VCU_ChassisGearFb) {
    case static_cast<int8_t>(VCU_CHASSISGEARFB_D):
      shift_msg.shift.data = autoware_vehicle_msgs::Shift::DRIVE;
      break;
    case static_cast<int8_t>(VCU_CHASSISGEARFB_N):
      shift_msg.shift.data = autoware_vehicle_msgs::Shift::NEUTRAL;
      break;
    case static_cast<int8_t>(VCU_CHASSISGEARFB_NO_USE):
      shift_msg.shift.data = autoware_vehicle_msgs::Shift::NONE;
      break;
    case static_cast<int8_t>(VCU_CHASSISGEARFB_R):
      shift_msg.shift.data = autoware_vehicle_msgs::Shift::REVERSE;
      break;
    default:
      shift_msg.shift.data = autoware_vehicle_msgs::Shift::NONE;
      break;
  }
  pub_shift_.publish(shift_msg);

  double linear_velocity = msg->VCU_ChassisSpeedFb;
  geometry_msgs::TwistStamped twist_msg;
  std_msgs::Float32 velocity_msg;

  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = msg->header.stamp;
  twist_msg.twist.linear.x = linear_velocity;
  pub_twist_.publish(twist_msg);

  velocity_msg.data = linear_velocity;
  pub_velocity_.publish(velocity_msg);
}

void ReportConverter::vehicleWorkCallback(
  const pix_hooke_driver_msgs::v2a_vehicleworkstafb_534ConstPtr & msg)
{
  autoware_vehicle_msgs::ControlMode control_mode_msg;
  control_mode_msg.header.stamp = msg->header.stamp;
  if (msg->VCU_DrivingModeFb == static_cast<int8_t>(VCU_DRIVINGMODEFB_SELF_DRIVING)) {
    control_mode_msg.data = autoware_vehicle_msgs::ControlMode::AUTO;
  } else {
    control_mode_msg.data = autoware_vehicle_msgs::ControlMode::MANUAL;
  }

  pub_control_mode_.publish(control_mode_msg);
}

}  // namespace report_converter

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "~");
  report_converter::ReportConverter report_converter_node;
  ros::spin();

  return 0;
}
