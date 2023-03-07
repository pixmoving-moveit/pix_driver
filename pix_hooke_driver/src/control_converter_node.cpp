/**
 * @file control_converter_node.cpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief convert autoware_control_msgs to pix_hooke_driver_msgs
 * @version 0.1
 * @date 2022-12-08
 * 
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 * 
 */

#include <memory>
#include <string>

#include "ros/ros.h"
// autoware
#include "autoware_vehicle_msgs/RawVehicleCommand.h"

// control
#include "pix_hooke_driver_msgs/a2v_brakectrl_131.h"
#include "pix_hooke_driver_msgs/a2v_drivectrl_130.h"
#include "pix_hooke_driver_msgs/a2v_steerctrl_132.h"
#include "pix_hooke_driver_msgs/a2v_vehiclectrl_133.h"
// report
#include "pix_hooke_driver_msgs/v2a_drivestafb_530.h"

#include "autoware_vehicle_msgs/RawVehicleCommand.h"
#include "autoware_vehicle_msgs/Shift.h"


namespace control_converter  // namespace control_converter
{
enum { ACU_CHASSISDRIVERENCTRL_DISABLE, ACU_CHASSISDRIVERENCTRL_ENABLE };

enum {
  ACU_CHASSISDRIVERMODECTRL_SPEED_CTRL_MODE,
  ACU_CHASSISDRIVERMODECTRL_THROTTLE_CTRL_MODE,
  ACU_CHASSISDRIVERMODECTRL_RESERVE,
};
enum {
  ACU_CHASSISGEARCTRL_DEFAULT_N,
  ACU_CHASSISGEARCTRL_D,
  ACU_CHASSISGEARCTRL_N,
  ACU_CHASSISGEARCTRL_R
};

enum { VCU_CHASSISGEARFB_NO_USE, VCU_CHASSISGEARFB_D, VCU_CHASSISGEARFB_N, VCU_CHASSISGEARFB_R };

enum {
  ACU_CHASSISSTEERMODECTRL_FRONT_ACKERMAN = 0,
  ACU_CHASSISSTEERMODECTRL_SAME_FRONT_AND_BACK,
  ACU_CHASSISSTEERMODECTRL_FRONT_DIFFERENT_BACK,
  ACU_CHASSISSTEERMODECTRL_BACK_ACKRMAN,
  ACU_CHASSISSTEERMODECTRL_FRONT_BACK
};

struct Params
{
  double control_period;      // hz
  double max_steering_angle;  // radians
  std::string sub_raw_command_topic;
  std::string sub_gear_report_topic;

  std::string pub_drive_topic;
  std::string pub_brake_topic;
  std::string pub_steer_topic;

};

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("CONTROL CONVERTER NODE is waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}

class ControlConverter
{
public:
  ControlConverter();
  ~ControlConverter();

  // msgs callbacks
  void gearReportCallback(const pix_hooke_driver_msgs::v2a_drivestafb_530ConstPtr & msg);
  void rawCommandCallback(const autoware_vehicle_msgs::RawVehicleCommandConstPtr & msg);

  // timer callbacks
  void timerCallback(const ros::TimerEvent & te);

private:
  Params control_converter_params_;
  double steering_factor_;

  std::shared_ptr<pix_hooke_driver_msgs::v2a_drivestafb_530> gear_report_ptr_;
  std::shared_ptr<autoware_vehicle_msgs::RawVehicleCommand> raw_vehicle_command_ptr_;
  std::shared_ptr<autoware_vehicle_msgs::RawVehicleCommand> prev_raw_vehicle_command_ptr_;

protected:
  // node handle
  ros::NodeHandle nh_;

  // publishers
  ros::Publisher pub_drive_;
  ros::Publisher pub_steer_;
  ros::Publisher pub_brake_;

  // subscribers
  ros::Subscriber sub_raw_vehicle_cmd_;
  ros::Subscriber sub_gear_report_;

  // timer
  ros::Timer timer_;
};

ControlConverter::ControlConverter()
{
  // params
  {
    control_converter_params_.max_steering_angle =
      waitForParam<double>(nh_, "/vehicle_info/max_steer_angle");
    // nh_.getParam("/vehicle_info/max_steer_angle", control_converter_params_.control_period);
    nh_.getParam("control_converter_node/control_period", control_converter_params_.control_period);
    nh_.getParam(
      "control_converter_node/sub_raw_command_topic",
      control_converter_params_.sub_raw_command_topic);
    nh_.getParam(
      "control_converter_node/sub_gear_report_topic",
      control_converter_params_.sub_gear_report_topic);
    nh_.getParam(
      "control_converter_node/pub_drive_topic", control_converter_params_.pub_drive_topic);
    nh_.getParam(
      "control_converter_node/pub_steer_topic", control_converter_params_.pub_steer_topic);
    nh_.getParam(
      "control_converter_node/pub_brake_topic", control_converter_params_.pub_brake_topic);

    steering_factor_ = 500.0 / (control_converter_params_.max_steering_angle);
  }
  // subscribers
  sub_raw_vehicle_cmd_ = nh_.subscribe(
    control_converter_params_.sub_raw_command_topic, 1, &ControlConverter::rawCommandCallback,
    this);
  sub_gear_report_ = nh_.subscribe(
    control_converter_params_.sub_gear_report_topic, 1, &ControlConverter::gearReportCallback,
    this);
  // publishers
  pub_drive_ = nh_.advertise<pix_hooke_driver_msgs::a2v_drivectrl_130>(
    control_converter_params_.pub_drive_topic, 5, false);
  pub_brake_ = nh_.advertise<pix_hooke_driver_msgs::a2v_brakectrl_131>(
    control_converter_params_.pub_brake_topic, 5, false);
  pub_steer_ = nh_.advertise<pix_hooke_driver_msgs::a2v_steerctrl_132>(
    control_converter_params_.pub_steer_topic, 5, false);
  // timers
  timer_ = nh_.createTimer(
    ros::Duration(1.0 / control_converter_params_.control_period),
    &ControlConverter::timerCallback, this);
}

ControlConverter::~ControlConverter() {}

void ControlConverter::gearReportCallback(
  const pix_hooke_driver_msgs::v2a_drivestafb_530ConstPtr & msg)
{
  gear_report_ptr_ = std::make_shared<pix_hooke_driver_msgs::v2a_drivestafb_530>(*msg);
}

void ControlConverter::rawCommandCallback(
  const autoware_vehicle_msgs::RawVehicleCommandConstPtr & msg)
{
  raw_vehicle_command_ptr_ = std::make_shared<autoware_vehicle_msgs::RawVehicleCommand>(*msg);
}

void ControlConverter::timerCallback(const ros::TimerEvent & te)
{
  if (gear_report_ptr_ == nullptr || raw_vehicle_command_ptr_ == nullptr) return;

  pix_hooke_driver_msgs::a2v_brakectrl_131 brake_cmd_msg;
  pix_hooke_driver_msgs::a2v_drivectrl_130 drive_cmd_msg;
  pix_hooke_driver_msgs::a2v_steerctrl_132 steer_cmd_msg;

  ros::Time stamp = ros::Time::now();

  // brake
  brake_cmd_msg.header.stamp = stamp;
  brake_cmd_msg.ACU_ChassisBrakePdlTarget = raw_vehicle_command_ptr_->control.brake * 100.0;
  brake_cmd_msg.ACU_ChassisBrakeEn = 1;

  // steer
  steer_cmd_msg.header.stamp = stamp;
  steer_cmd_msg.ACU_ChassisSteerAngleSpeedCtrl = 250;
  steer_cmd_msg.ACU_ChassisSteerAngleTarget =
    -raw_vehicle_command_ptr_->control.steering_angle * steering_factor_;
  steer_cmd_msg.ACU_ChassisSteerEnCtrl = 1;
  steer_cmd_msg.ACU_ChassisSteerModeCtrl =
    static_cast<int8_t>(ACU_CHASSISSTEERMODECTRL_FRONT_DIFFERENT_BACK);

  // gear
  drive_cmd_msg.header.stamp = stamp;

  switch (raw_vehicle_command_ptr_->shift.data) {
    case autoware_vehicle_msgs::Shift::NONE:
      drive_cmd_msg.ACU_ChassisGearCtrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_DEFAULT_N);
      break;
    case autoware_vehicle_msgs::Shift::DRIVE:
      drive_cmd_msg.ACU_ChassisGearCtrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_D);
      break;
    case autoware_vehicle_msgs::Shift::NEUTRAL:
      drive_cmd_msg.ACU_ChassisGearCtrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_N);
      break;
    case autoware_vehicle_msgs::Shift::REVERSE:
      drive_cmd_msg.ACU_ChassisGearCtrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_R);
      break;
    default:
      drive_cmd_msg.ACU_ChassisGearCtrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_N);
      break;
  }

  // throttle
  drive_cmd_msg.ACU_ChassisDriverEnCtrl = ACU_CHASSISDRIVERENCTRL_ENABLE;
  drive_cmd_msg.ACU_ChassisDriverModeCtrl = ACU_CHASSISDRIVERMODECTRL_THROTTLE_CTRL_MODE;
  drive_cmd_msg.ACU_ChassisThrottlePdlTarget = 2.0 * raw_vehicle_command_ptr_->control.throttle * 100.0;

  // keep shifting when target gear is different from actual gear
  if (gear_report_ptr_->VCU_ChassisGearFb != drive_cmd_msg.ACU_ChassisGearCtrl) {
    brake_cmd_msg.ACU_ChassisBrakePdlTarget = 20.0;
    drive_cmd_msg.ACU_ChassisThrottlePdlTarget = 0.0;
  }

  // when we are not receiving raw_vehicle_msg for 0.1s, this node will not publishing msgs
  if (ros::Time::now().toNSec() - raw_vehicle_command_ptr_->header.stamp.toNSec() > 10000000)
    return;
  // publishing
  pub_brake_.publish(brake_cmd_msg);
  pub_drive_.publish(drive_cmd_msg);
  pub_steer_.publish(steer_cmd_msg);
}

}  // namespace control_converter

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "~");
  control_converter::ControlConverter control_converter_node;
  ros::spin();

  return 0;
}
