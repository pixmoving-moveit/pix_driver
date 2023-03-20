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

#include <pix_hooke_driver/control_command.hpp>

namespace pix_hooke_driver
{
namespace control_command
{
ControlCommand::ControlCommand() : Node("control_command")
{
  // ros params
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");
  param_.command_timeout_ms = declare_parameter("command_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);

  // initialize msg received time, make reservation of data size
  brake_command_received_time_ = this->now();
  drive_command_received_time_ = this->now();
  steer_command_received_time_ = this->now();
  wheel_command_received_time_ = this->now();
  vehicle_command_received_time_ = this->now();

  // // initialize can msgs
  // brake_ctrl_can_ptr_->data;
  // drive_ctrl_can_ptr_->data;
  // steer_ctrl_can_ptr_->data;
  // wheel_ctrl_can_ptr_->data;
  // vehicle_strl_can_ptr_->data;

  is_engage_ = true;

  using std::placeholders::_1;

  /* subscriber */
  {
    // from pix driver autoware interface
    a2v_brake_ctrl_sub_ = create_subscription<A2vBrakeCtrl>(
      "/pix_hooke/a2v_brakectrl_131", 1, std::bind(&ControlCommand::callbackBrakeCtrl, this, _1));
    a2v_drive_ctrl_sub_ = create_subscription<A2vDriveCtrl>(
      "/pix_hooke/a2v_drivectrl_130", 1, std::bind(&ControlCommand::callbackDriveCtrl, this, _1));
    a2v_steer_ctrl_sub_ = create_subscription<A2vSteerCtrl>(
      "/pix_hooke/a2v_steerctrl_132", 1, std::bind(&ControlCommand::callbackSteerCtrl, this, _1));
    a2v_wheel_ctrl_sub_ = create_subscription<A2vWheelCtrl>(
      "/pix_hooke/a2v_wheelctrl_135", 1, std::bind(&ControlCommand::callbackWheelCtrl, this, _1));
    a2v_vehicle_ctrl_sub_ = create_subscription<A2vVehicleCtrl>(
      "/pix_hooke/a2v_vehiclectrl_133", 1, std::bind(&ControlCommand::callbackVehicleCtrl, this, _1));
    // engage
    engage_ctrl_sub_ = create_subscription<std_msgs::msg::Bool>(
      "input/engage", 1, std::bind(&ControlCommand::callbackEngage, this, _1));
  }
  /* publisher */
  {
    // to socketcan drivier
    can_frame_pub_ = create_publisher<can_msgs::msg::Frame>("output/can_tx", rclcpp::QoS{1});
  }
  {
    // timer
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
      std::bind(&ControlCommand::timerCallback, this));
  }
}

void ControlCommand::callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg)
{
  brake_command_received_time_ = this->now();
  brake_ctrl_ptr_ = msg;
  a2v_brakectrl_131_entity_.Reset();
  a2v_brakectrl_131_entity_.UpdateData(
    msg->acu_chassis_brake_en, msg->acu_chassis_aeb_ctrl, msg->acu_chassis_brake_pdl_target,
    msg->acu_chassis_epb_ctrl, msg->acu_brake_life_sig, msg->acu_check_sum_131);
  can_msgs::msg::Frame brake_ctrl_can_msg;
  brake_ctrl_can_msg.header.stamp = msg->header.stamp;
  brake_ctrl_can_msg.dlc = 8;
  brake_ctrl_can_msg.id = a2v_brakectrl_131_entity_.ID;
  brake_ctrl_can_msg.is_extended = false;
  uint8_t *signal_bits;
  signal_bits = a2v_brakectrl_131_entity_.get_data();
  for (int i = 0; i < 8; i++)
  {
    brake_ctrl_can_msg.data[i] = *signal_bits;
    signal_bits += 1;
  }
  brake_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(brake_ctrl_can_msg);
}

void ControlCommand::callbackDriveCtrl(const A2vDriveCtrl::ConstSharedPtr & msg)
{
  drive_command_received_time_ = this->now();
  drive_ctrl_ptr_ = msg;
  a2v_drivectrl_130_entity_.Reset();
  a2v_drivectrl_130_entity_.UpdateData(
    msg->acu_chassis_driver_en_ctrl, msg->acu_chassis_driver_mode_ctrl, msg->acu_chassis_gear_ctrl,
    msg->acu_chassis_speed_ctrl, msg->acu_chassis_throttle_pdl_target, msg->acu_drive_life_sig,
    msg->acu_check_sum_130);
  can_msgs::msg::Frame drive_ctrl_can_msg;
  drive_ctrl_can_msg.header.stamp = msg->header.stamp;
  drive_ctrl_can_msg.dlc = 8;
  drive_ctrl_can_msg.id = a2v_drivectrl_130_entity_.ID;
  drive_ctrl_can_msg.is_extended = false;
  uint8_t *signal_bits;
  signal_bits = a2v_drivectrl_130_entity_.get_data();
  for (int i = 0; i < 8; i++)
  {
    drive_ctrl_can_msg.data[i] = *signal_bits;
    signal_bits += 1;
  }
  drive_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(drive_ctrl_can_msg); 
}

void ControlCommand::callbackSteerCtrl(const A2vSteerCtrl::ConstSharedPtr & msg)
{
  steer_command_received_time_ = this->now();
  steer_ctrl_ptr_ = msg;
  a2v_steerctrl_132_entity_.Reset();
  a2v_steerctrl_132_entity_.UpdateData(
    msg->acu_chassis_steer_en_ctrl, msg->acu_chassis_steer_mode_ctrl,
    msg->acu_chassis_steer_angle_target, msg->acu_chassis_steer_angle_rear_target,
    msg->acu_chassis_steer_angle_speed_ctrl, msg->acu_check_sum_132);
  can_msgs::msg::Frame steer_ctrl_can_msg;
  steer_ctrl_can_msg.header.stamp = msg->header.stamp;
  steer_ctrl_can_msg.dlc = 8;
  steer_ctrl_can_msg.id = a2v_steerctrl_132_entity_.ID;
  steer_ctrl_can_msg.is_extended = false;
  uint8_t *signal_bits;
  signal_bits = a2v_steerctrl_132_entity_.get_data();
  for (int i = 0; i < 8; i++)
  {
    steer_ctrl_can_msg.data[i] = *signal_bits;
    signal_bits += 1;
  }
  steer_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(steer_ctrl_can_msg);
}

void ControlCommand::callbackWheelCtrl(const A2vWheelCtrl::ConstSharedPtr & msg)
{
  wheel_command_received_time_ = this->now();
  wheel_ctrl_ptr_ = msg;
  a2v_wheelctrl_135_entity_.Reset();
  a2v_wheelctrl_135_entity_.UpdateData(
    msg->acu_motor_torque_lf_crtl, msg->acu_motor_torque_rf_crtl, msg->acu_motor_torque_lr_crtl,
    msg->acu_motor_torque_rr_crtl);
  can_msgs::msg::Frame wheel_ctrl_can_msg;
  wheel_ctrl_can_msg.header.stamp = msg->header.stamp;
  wheel_ctrl_can_msg.dlc = 8;
  wheel_ctrl_can_msg.id = a2v_wheelctrl_135_entity_.ID;
  wheel_ctrl_can_msg.is_extended = false;
  uint8_t *signal_bits;
  signal_bits = a2v_wheelctrl_135_entity_.get_data();
  for (int i = 0; i < 8; i++)
  {
    wheel_ctrl_can_msg.data[i] = *signal_bits;
    signal_bits += 1;
  }
  wheel_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(wheel_ctrl_can_msg);
}
void ControlCommand::callbackVehicleCtrl(const A2vVehicleCtrl::ConstSharedPtr & msg)
{

  vehicle_command_received_time_ = this->now();
  vehicle_ctrl_ptr_ = msg;
  a2v_vehiclectrl_133_entity_.Reset();
  a2v_vehiclectrl_133_entity_.UpdateData(
    msg->acu_vehicle_pos_lamp_ctrl, msg->acu_vehicle_head_lamp_ctrl,
    msg->acu_vehicle_left_lamp_ctrl, msg->acu_vehicle_right_lamp_ctrl,
    msg->acu_vehicl_high_beam_ctrl, msg->acu_vehicle_fog_lamp_ctrl,
    msg->acu_vehicle_body_light_crtl, msg->acu_vehicle_read_light_crtl, msg->acu_vehicle_voice,
    msg->acu_vehicle_wipers_crtl, msg->acu_vehicle_door_crtl, msg->acu_vehicle_window_crtl,
    msg->acu_chassis_speed_limite_mode, msg->acu_chassis_speed_limite_val, msg->acu_check_sum_en);
  can_msgs::msg::Frame vehicle_ctrl_can_msg;
  vehicle_ctrl_can_msg.header.stamp = msg->header.stamp;
  vehicle_ctrl_can_msg.dlc = 8;
  vehicle_ctrl_can_msg.id = a2v_vehiclectrl_133_entity_.ID;
  vehicle_ctrl_can_msg.is_extended = false;
  uint8_t *signal_bits;
  signal_bits = a2v_wheelctrl_135_entity_.get_data();
  for (int i = 0; i < 8; i++)
  {
    vehicle_ctrl_can_msg.data[i] = *signal_bits;
    signal_bits += 1;
  }
  vehicle_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(vehicle_ctrl_can_msg);
}

void ControlCommand::callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_engage_ = msg->data;
}

void ControlCommand::timerCallback()
{
  if (!is_engage_) return;
  const rclcpp::Time current_time = this->now();

  // brake control command 
  const double brake_command_delta_time_ms =
    (current_time - brake_command_received_time_).seconds() * 1000.0;
  if (brake_command_delta_time_ms > param_.command_timeout_ms || brake_ctrl_can_ptr_==nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "brake command timeout = %f ms.", brake_command_delta_time_ms);
  } else {
    can_frame_pub_->publish(*brake_ctrl_can_ptr_);
  }
  // drive control command
  const double drive_command_delta_time_ms =
    (current_time - drive_command_received_time_).seconds() * 1000.0;
  if(drive_command_delta_time_ms > param_.command_timeout_ms || drive_ctrl_can_ptr_==nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "drive command timeout = %f ms.", drive_command_delta_time_ms);
  } else {
    can_frame_pub_->publish(*drive_ctrl_can_ptr_);
  }
  // steer control command
  const double steer_command_delta_time_ms =
    (current_time - steer_command_received_time_).seconds() * 1000.0;
  if(steer_command_delta_time_ms > param_.command_timeout_ms || steer_ctrl_can_ptr_==nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "steer command timeout = %f ms.", steer_command_delta_time_ms);
  } else {
    can_frame_pub_->publish(*steer_ctrl_can_ptr_);
  }
  // wheel control command
  const double wheel_command_delta_time_ms =
    (current_time - wheel_command_received_time_).seconds() * 1000.0;
  if(wheel_command_delta_time_ms > param_.command_timeout_ms || wheel_ctrl_can_ptr_==nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "wheel command timeout = %f ms.", wheel_command_delta_time_ms);
  } else {
    can_frame_pub_->publish(*wheel_ctrl_can_ptr_);
  }
  // vehicle control command
  const double vehicle_command_delta_time_ms =
    (current_time - vehicle_command_received_time_).seconds() * 1000.0;
  if(vehicle_command_delta_time_ms > param_.command_timeout_ms || vehicle_ctrl_can_ptr_==nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "vehicle command timeout = %f ms.", vehicle_command_delta_time_ms);
  } else {
    can_frame_pub_->publish(*vehicle_ctrl_can_ptr_);
  }
}
} // namespace control_command
} // namespace pix_hooke_driver