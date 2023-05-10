#ifndef PIX_ROBOBUS_DRIVER__CONTROL_COMMAND_HPP_
#define PIX_ROBOBUS_DRIVER__CONTROL_COMMAND_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <can_msgs/msg/frame.hpp>


// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include <pix_robobus_driver_msgs/msg/throttle_command.hpp>
#include <pix_robobus_driver_msgs/msg/brake_command.hpp>
#include <pix_robobus_driver_msgs/msg/steering_command.hpp>
#include <pix_robobus_driver_msgs/msg/gear_command.hpp>
#include <pix_robobus_driver_msgs/msg/park_command.hpp>
#include <pix_robobus_driver_msgs/msg/vehicle_mode_command.hpp>



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include <pix_robobus_driver/throttle_command.hpp>
#include <pix_robobus_driver/brake_command.hpp>
#include <pix_robobus_driver/steering_command.hpp>
#include <pix_robobus_driver/gear_command.hpp>
#include <pix_robobus_driver/park_command.hpp>
#include <pix_robobus_driver/vehicle_mode_command.hpp>


namespace pix_robobus_driver
{
namespace control_command
{

/**
 * @brief param structure of control command node
 * @param base_frame_id frame id of vehicle
 * @param loop_rate loop rate of publishers in hz
 * @param command_timeout_ms timeout threshold of control command msg from control converter in ms
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
  // parameters of node
  Param param_;

  // subscribers
  // example rclcpp::Subscription<A2vBrakeCtrl>::SharedPtr a2v_brake_ctrl_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::ThrottleCommand>::SharedPtr throttle_command_sub_;
rclcpp::Subscription<pix_robobus_driver_msgs::msg::BrakeCommand>::SharedPtr brake_command_sub_;
rclcpp::Subscription<pix_robobus_driver_msgs::msg::SteeringCommand>::SharedPtr steering_command_sub_;
rclcpp::Subscription<pix_robobus_driver_msgs::msg::GearCommand>::SharedPtr gear_command_sub_;
rclcpp::Subscription<pix_robobus_driver_msgs::msg::ParkCommand>::SharedPtr park_command_sub_;
rclcpp::Subscription<pix_robobus_driver_msgs::msg::VehicleModeCommand>::SharedPtr vehicle_mode_command_sub_;


  // msgs
  // example A2vBrakeCtrl::ConstSharedPtr brake_ctrl_ptr_;
  pix_robobus_driver_msgs::msg::ThrottleCommand::ConstSharedPtr throttle_command_ptr_;
pix_robobus_driver_msgs::msg::BrakeCommand::ConstSharedPtr brake_command_ptr_;
pix_robobus_driver_msgs::msg::SteeringCommand::ConstSharedPtr steering_command_ptr_;
pix_robobus_driver_msgs::msg::GearCommand::ConstSharedPtr gear_command_ptr_;
pix_robobus_driver_msgs::msg::ParkCommand::ConstSharedPtr park_command_ptr_;
pix_robobus_driver_msgs::msg::VehicleModeCommand::ConstSharedPtr vehicle_mode_command_ptr_;


  // control command structures
  // example A2vdrivectrl130 a2v_drivectrl_130_entity_;
  ThrottleCommand throttle_command_entity_;
BrakeCommand brake_command_entity_;
SteeringCommand steering_command_entity_;
GearCommand gear_command_entity_;
ParkCommand park_command_entity_;
VehicleModeCommand vehicle_mode_command_entity_;


  // msg received timestamp
  // example rclcpp::Time drive_command_received_time_;
  rclcpp::Time throttle_command_received_time_;
rclcpp::Time brake_command_received_time_;
rclcpp::Time steering_command_received_time_;
rclcpp::Time gear_command_received_time_;
rclcpp::Time park_command_received_time_;
rclcpp::Time vehicle_mode_command_received_time_;


  // state control
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_ctrl_sub_;
  bool is_engage_;

  // publishers to can card driver
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  // publishing can msgs
  // example can_msgs::msg::Frame::ConstSharedPtr brake_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr throttle_command_can_ptr_;
can_msgs::msg::Frame::ConstSharedPtr brake_command_can_ptr_;
can_msgs::msg::Frame::ConstSharedPtr steering_command_can_ptr_;
can_msgs::msg::Frame::ConstSharedPtr gear_command_can_ptr_;
can_msgs::msg::Frame::ConstSharedPtr park_command_can_ptr_;
can_msgs::msg::Frame::ConstSharedPtr vehicle_mode_command_can_ptr_;


  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ControlCommand();
  // calback functions
  // example
  // void callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg);
  void callbackThrottleCommand(const pix_robobus_driver_msgs::msg::ThrottleCommand::ConstSharedPtr & msg);

void callbackBrakeCommand(const pix_robobus_driver_msgs::msg::BrakeCommand::ConstSharedPtr & msg);

void callbackSteeringCommand(const pix_robobus_driver_msgs::msg::SteeringCommand::ConstSharedPtr & msg);

void callbackGearCommand(const pix_robobus_driver_msgs::msg::GearCommand::ConstSharedPtr & msg);

void callbackParkCommand(const pix_robobus_driver_msgs::msg::ParkCommand::ConstSharedPtr & msg);

void callbackVehicleModeCommand(const pix_robobus_driver_msgs::msg::VehicleModeCommand::ConstSharedPtr & msg);


  void callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void timerCallback();

};

} // control_command
} // pix_robobus_driver
#endif // PIX_ROBOBUS_DRIVER__CONTROL_COMMAND_HPP_