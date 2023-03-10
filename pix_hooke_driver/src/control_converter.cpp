#include <memory>
#include <string>

#include <pix_hooke_driver/control_converter.hpp>

namespace pix_hooke_driver
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

  // initialize msgs and timestamps
  drive_sta_fb_received_time_ = this->now();
  actuation_command_received_time_ = this->now();
  gear_command_received_time_ = this->now();

  // publishers
  a2v_brake_ctrl_pub_ =
    create_publisher<A2vBrakeCtrl>("/pix_hooke/a2v_brakectrl_131", rclcpp::QoS(1));
  a2v_drive_ctrl_pub_ =
    create_publisher<A2vDriveCtrl>("/pix_hooke/a2v_drivectrl_130", rclcpp::QoS(1));
  a2v_steer_ctrl_pub_ =
    create_publisher<A2vSteerCtrl>("/pix_hooke/a2v_steerctrl_132", rclcpp::QoS(1));
  a2v_vehicle_ctrl_pub_ =
    create_publisher<A2vVehicleCtrl>("/pix_hooke/a2v_vehiclectrl_133", rclcpp::QoS(1));

  // subscribers
  actuation_command_sub_ =
    create_subscription<tier4_vehicle_msgs::msg::ActuationCommand>(
      "/control/command/actuation_cmd", 1,
      std::bind(&ControlConverter::callbackActuationCommand, this, std::placeholders::_1));
  gear_command_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1,
    std::bind(&ControlConverter::callbackGearCommand, this, std::placeholders::_1));
  drive_feedback_sub_ = create_subscription<V2aDriveStaFb>(
    "/pix_hooke/v2a_drivestafb", 1,
    std::bind(&ControlConverter::callbackDriveStatusFeedback, this, std::placeholders::_1));
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
    std::bind(&ControlConverter::timerCallback, this));
}

void ControlConverter::callbackActuationCommand(
  const tier4_vehicle_msgs::msg::ActuationCommand::ConstSharedPtr & msg)
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

void ControlConverter::callbackDriveStatusFeedback(const V2aDriveStaFb::ConstSharedPtr & msg)
{
  drive_sta_fb_received_time_ = this->now();
  drive_sta_fb_ptr_ = msg;
}

void ControlConverter::timerCallback()
{
  const rclcpp::Time current_time = this->now();
  const double actuation_command_delta_time_ms =
    (current_time - actuation_command_received_time_).seconds() * 1000.0;
  const double gear_command_delta_time_ms =
    (current_time - gear_command_received_time_).seconds() * 1000.0;
  const double drive_sta_fb_delta_time_ms =
    (current_time - drive_sta_fb_received_time_).seconds() * 1000.0;

  if(actuation_command_delta_time_ms > param_.autoware_control_command_timeout
      || gear_command_delta_time_ms > param_.autoware_control_command_timeout
      || actuation_command_ptr_==nullptr
      || gear_command_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "actuation command timeout = %f ms, gear command timeout = %f",
      actuation_command_delta_time_ms, gear_command_delta_time_ms);
    return;
  }
  if(drive_sta_fb_delta_time_ms > param_.autoware_control_command_timeout || drive_sta_fb_ptr_ == nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "gear feedback timeout = %f ms.", drive_sta_fb_delta_time_ms);
    return;
  }

  // sending control messages to pix dirver control command
  A2vBrakeCtrl a2v_brake_ctrl_msg;
  A2vDriveCtrl a2v_drive_ctrl_msg;
  A2vSteerCtrl a2v_steer_ctrl_msg;
  A2vVehicleCtrl a2v_vehicle_ctrl_msg;

  // brake
  a2v_brake_ctrl_msg.header.stamp = current_time;
  a2v_brake_ctrl_msg.acu_chassis_brake_pdl_target = actuation_command_ptr_->brake_cmd * 100.0;
  a2v_brake_ctrl_msg.acu_chassis_brake_en = 1;

  // steer
  a2v_steer_ctrl_msg.header.stamp = current_time;
  a2v_steer_ctrl_msg.acu_chassis_steer_angle_speed_ctrl = 250;
  a2v_steer_ctrl_msg.acu_chassis_steer_angle_target =
    -actuation_command_ptr_->steer_cmd * steering_factor_;
  a2v_steer_ctrl_msg.acu_chassis_steer_en_ctrl = 1;
  a2v_steer_ctrl_msg.acu_chassis_steer_mode_ctrl =
    static_cast<int8_t>(ACU_CHASSISSTEERMODECTRL_FRONT_DIFFERENT_BACK);

  // gear
  a2v_drive_ctrl_msg.header.stamp = current_time;
  switch (gear_command_ptr_->command) {
    case autoware_auto_vehicle_msgs::msg::GearCommand::NONE:
      a2v_drive_ctrl_msg.acu_chassis_gear_ctrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_DEFAULT_N);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
      a2v_drive_ctrl_msg.acu_chassis_gear_ctrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_D);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL:
      a2v_drive_ctrl_msg.acu_chassis_gear_ctrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_N);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
      a2v_drive_ctrl_msg.acu_chassis_gear_ctrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_R);
      break;
    default:
      a2v_drive_ctrl_msg.acu_chassis_gear_ctrl = static_cast<int8_t>(ACU_CHASSISGEARCTRL_N);
      break;
  }

  // throttle
  a2v_drive_ctrl_msg.acu_chassis_driver_en_ctrl = ACU_CHASSISDRIVERENCTRL_ENABLE;
  a2v_drive_ctrl_msg.acu_chassis_driver_mode_ctrl = ACU_CHASSISDRIVERMODECTRL_THROTTLE_CTRL_MODE;
  a2v_drive_ctrl_msg.acu_chassis_throttle_pdl_target = actuation_command_ptr_->accel_cmd * 100.0;

  // keep shifting and braking when target gear is different from actual gear
  if (drive_sta_fb_ptr_->vcu_chassis_gear_fb != a2v_drive_ctrl_msg.acu_chassis_gear_ctrl) {
    a2v_brake_ctrl_msg.acu_chassis_brake_pdl_target = 20.0;
    a2v_drive_ctrl_msg.acu_chassis_throttle_pdl_target = 0.0;
  }
  // publishing msgs
  a2v_brake_ctrl_pub_->publish(a2v_brake_ctrl_msg);
  a2v_drive_ctrl_pub_->publish(a2v_drive_ctrl_msg);
  a2v_steer_ctrl_pub_->publish(a2v_steer_ctrl_msg);
}
} // namespace control_converter
} // namespace pix_driver
