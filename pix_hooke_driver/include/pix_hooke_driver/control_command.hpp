#ifndef PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_
#define PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <can_msgs/msg/frame.hpp>

#include <pix_hooke_driver_msgs/msg/a2v_brake_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_drive_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_steer_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_vehicle_ctrl.hpp>
#include <pix_hooke_driver_msgs/msg/a2v_wheel_ctrl.hpp>

#include <pix_hooke_driver/a2v_brakectrl_131.hpp>
#include <pix_hooke_driver/a2v_drivectrl_130.hpp>
#include <pix_hooke_driver/a2v_steerctrl_132.hpp>
#include <pix_hooke_driver/a2v_vehiclectrl_133.hpp>
#include <pix_hooke_driver/a2v_wheelctrl_135.hpp>

#include <string>
#include <memory>

namespace pix_hooke_driver
{
using A2vBrakeCtrl = pix_hooke_driver_msgs::msg::A2vBrakeCtrl;
using A2vDriveCtrl = pix_hooke_driver_msgs::msg::A2vDriveCtrl;
using A2vSteerCtrl = pix_hooke_driver_msgs::msg::A2vSteerCtrl;
using A2vWheelCtrl = pix_hooke_driver_msgs::msg::A2vWheelCtrl;
using A2vVehicleCtrl = pix_hooke_driver_msgs::msg::A2vVehicleCtrl;

/**
 * @brief parameters structure
 * 
 */
struct Param
{
  std::string base_frame_id;
  double loop_rate;
  int command_timeout_ms;
};

class ControlCommand : public rclcpp::Node
{
private:
  // parameter
  Param param_;

  // subscribers from pix driver autoware interface
  rclcpp::Subscription<A2vBrakeCtrl>::SharedPtr a2v_brake_ctrl_sub_;
  rclcpp::Subscription<A2vDriveCtrl>::SharedPtr a2v_drive_ctrl_sub_;
  rclcpp::Subscription<A2vSteerCtrl>::SharedPtr a2v_steer_ctrl_sub_;
  rclcpp::Subscription<A2vWheelCtrl>::SharedPtr a2v_wheel_ctrl_sub_;
  rclcpp::Subscription<A2vVehicleCtrl>::SharedPtr a2v_vehicle_ctrl_sub_;

  A2vBrakeCtrl::ConstSharedPtr brake_ctrl_ptr_;
  A2vDriveCtrl::ConstSharedPtr drive_ctrl_ptr_;
  A2vSteerCtrl::ConstSharedPtr steer_ctrl_ptr_;
  A2vWheelCtrl::ConstSharedPtr wheel_ctrl_ptr_;
  A2vVehicleCtrl::ConstSharedPtr vehicle_ctrl_ptr_;

  // control command structures
  A2vdrivectrl130 a2v_drivectrl_130_entity_;
  A2vbrakectrl131 a2v_brakectrl_131_entity_;
  A2vsteerctrl132 a2v_steerctrl_132_entity_;
  A2vvehiclectrl133 a2v_vehiclectrl_133_entity_;
  A2vwheelctrl135 a2v_wheelctrl_135_entity_;

  rclcpp::Time brake_command_received_time_;
  rclcpp::Time drive_command_received_time_;
  rclcpp::Time steer_command_received_time_;
  rclcpp::Time wheel_command_received_time_;
  rclcpp::Time vehicle_command_received_time_;

  // state control
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_ctrl_sub_;
  bool is_engage_;

  // publishers to can card driver
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  // publishing can msgs
  can_msgs::msg::Frame::ConstSharedPtr brake_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr drive_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr steer_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr wheel_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr vehicle_ctrl_can_ptr_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ControlCommand();

  void callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg);
  void callbackDriveCtrl(const A2vDriveCtrl::ConstSharedPtr & msg);
  void callbackSteerCtrl(const A2vSteerCtrl::ConstSharedPtr & msg);
  void callbackWheelCtrl(const A2vWheelCtrl::ConstSharedPtr & msg);
  void callbackVehicleCtrl(const A2vVehicleCtrl::ConstSharedPtr & msg);

  void callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg);

  void timerCallback();
};

} // namespace pix_hooke_driver
#endif // PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_