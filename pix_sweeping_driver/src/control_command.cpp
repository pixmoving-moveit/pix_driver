#include <pix_sweeping_driver/control_command.hpp>

namespace pix_sweeping_driver
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
  // example brake_command_received_time_ = this->now();
  throttle_command_received_time_ = this->now();brake_command_received_time_ = this->now();steering_command_received_time_ = this->now();gear_command_received_time_ = this->now();park_command_received_time_ = this->now();vehicle_mode_command_received_time_ = this->now();acu_sweep_ctrl_cmd_received_time_ = this->now();

  is_engage_ = true;

  using std::placeholders::_1;

  /* subscriber */
  {
    // from sweeping driver autoware interface
    /**
    a2v_brake_ctrl_sub_ = create_subscription<A2vBrakeCtrl>(
      "/pix_hooke/a2v_brakectrl_131", 1, std::bind(&ControlCommand::callbackBrakeCtrl, this, _1));
    **/
    throttle_command_sub_ = create_subscription<pix_sweeping_driver_msgs::msg::ThrottleCommand>("/pix_sweeping/throttle_command", 1, std::bind(&ControlCommand::callbackThrottleCommand, this, _1));
brake_command_sub_ = create_subscription<pix_sweeping_driver_msgs::msg::BrakeCommand>("/pix_sweeping/brake_command", 1, std::bind(&ControlCommand::callbackBrakeCommand, this, _1));
steering_command_sub_ = create_subscription<pix_sweeping_driver_msgs::msg::SteeringCommand>("/pix_sweeping/steering_command", 1, std::bind(&ControlCommand::callbackSteeringCommand, this, _1));
gear_command_sub_ = create_subscription<pix_sweeping_driver_msgs::msg::GearCommand>("/pix_sweeping/gear_command", 1, std::bind(&ControlCommand::callbackGearCommand, this, _1));
park_command_sub_ = create_subscription<pix_sweeping_driver_msgs::msg::ParkCommand>("/pix_sweeping/park_command", 1, std::bind(&ControlCommand::callbackParkCommand, this, _1));
vehicle_mode_command_sub_ = create_subscription<pix_sweeping_driver_msgs::msg::VehicleModeCommand>("/pix_sweeping/vehicle_mode_command", 1, std::bind(&ControlCommand::callbackVehicleModeCommand, this, _1));
acu_sweep_ctrl_cmd_sub_ = create_subscription<pix_sweeping_driver_msgs::msg::AcuSweepCtrlCmd>("/pix_sweeping/acu_sweep_ctrl_cmd", 1, std::bind(&ControlCommand::callbackAcuSweepCtrlCmd, this, _1));

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

// calback functions
/** example
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
**/


    void ControlCommand::callbackThrottleCommand(const pix_sweeping_driver_msgs::msg::ThrottleCommand::ConstSharedPtr & msg)
    {
        throttle_command_received_time_ = this->now();
        throttle_command_ptr_ = msg;
        throttle_command_entity_.Reset();
        throttle_command_entity_.UpdateData(msg->check_sum100, msg->dirve_speed_target, msg->dirve_throttle_pedal_target, msg->dirve_en_ctrl);
        can_msgs::msg::Frame throttle_command_can_msg;
        throttle_command_can_msg.header.stamp = msg->header.stamp;
        throttle_command_can_msg.dlc = 8;
        throttle_command_can_msg.id = throttle_command_entity_.ID;
        throttle_command_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = throttle_command_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            throttle_command_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        throttle_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(throttle_command_can_msg);
    }

    

    void ControlCommand::callbackBrakeCommand(const pix_sweeping_driver_msgs::msg::BrakeCommand::ConstSharedPtr & msg)
    {
        brake_command_received_time_ = this->now();
        brake_command_ptr_ = msg;
        brake_command_entity_.Reset();
        brake_command_entity_.UpdateData(msg->check_sum101, msg->brake_pedal_target, msg->brake_en_ctrl);
        can_msgs::msg::Frame brake_command_can_msg;
        brake_command_can_msg.header.stamp = msg->header.stamp;
        brake_command_can_msg.dlc = 8;
        brake_command_can_msg.id = brake_command_entity_.ID;
        brake_command_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = brake_command_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            brake_command_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        brake_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(brake_command_can_msg);
    }

    

    void ControlCommand::callbackSteeringCommand(const pix_sweeping_driver_msgs::msg::SteeringCommand::ConstSharedPtr & msg)
    {
        steering_command_received_time_ = this->now();
        steering_command_ptr_ = msg;
        steering_command_entity_.Reset();
        steering_command_entity_.UpdateData(msg->check_sum102, msg->steer_angle_target, msg->steer_angle_speed, msg->steer_en_ctrl);
        can_msgs::msg::Frame steering_command_can_msg;
        steering_command_can_msg.header.stamp = msg->header.stamp;
        steering_command_can_msg.dlc = 8;
        steering_command_can_msg.id = steering_command_entity_.ID;
        steering_command_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = steering_command_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            steering_command_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        steering_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(steering_command_can_msg);
    }

    

    void ControlCommand::callbackGearCommand(const pix_sweeping_driver_msgs::msg::GearCommand::ConstSharedPtr & msg)
    {
        gear_command_received_time_ = this->now();
        gear_command_ptr_ = msg;
        gear_command_entity_.Reset();
        gear_command_entity_.UpdateData(msg->check_sum103, msg->gear_target, msg->gear_en_ctrl);
        can_msgs::msg::Frame gear_command_can_msg;
        gear_command_can_msg.header.stamp = msg->header.stamp;
        gear_command_can_msg.dlc = 8;
        gear_command_can_msg.id = gear_command_entity_.ID;
        gear_command_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = gear_command_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            gear_command_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        gear_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(gear_command_can_msg);
    }

    

    void ControlCommand::callbackParkCommand(const pix_sweeping_driver_msgs::msg::ParkCommand::ConstSharedPtr & msg)
    {
        park_command_received_time_ = this->now();
        park_command_ptr_ = msg;
        park_command_entity_.Reset();
        park_command_entity_.UpdateData(msg->check_sum104, msg->park_target, msg->park_en_ctrl);
        can_msgs::msg::Frame park_command_can_msg;
        park_command_can_msg.header.stamp = msg->header.stamp;
        park_command_can_msg.dlc = 8;
        park_command_can_msg.id = park_command_entity_.ID;
        park_command_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = park_command_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            park_command_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        park_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(park_command_can_msg);
    }

    

    void ControlCommand::callbackVehicleModeCommand(const pix_sweeping_driver_msgs::msg::VehicleModeCommand::ConstSharedPtr & msg)
    {
        vehicle_mode_command_received_time_ = this->now();
        vehicle_mode_command_ptr_ = msg;
        vehicle_mode_command_entity_.Reset();
        vehicle_mode_command_entity_.UpdateData(msg->check_sum_105, msg->headlight_ctrl, msg->turn_light_ctrl, msg->drive_mode_ctrl, msg->steer_mode_ctrl);
        can_msgs::msg::Frame vehicle_mode_command_can_msg;
        vehicle_mode_command_can_msg.header.stamp = msg->header.stamp;
        vehicle_mode_command_can_msg.dlc = 8;
        vehicle_mode_command_can_msg.id = vehicle_mode_command_entity_.ID;
        vehicle_mode_command_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = vehicle_mode_command_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            vehicle_mode_command_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        vehicle_mode_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(vehicle_mode_command_can_msg);
    }

    

    void ControlCommand::callbackAcuSweepCtrlCmd(const pix_sweeping_driver_msgs::msg::AcuSweepCtrlCmd::ConstSharedPtr & msg)
    {
        acu_sweep_ctrl_cmd_received_time_ = this->now();
        acu_sweep_ctrl_cmd_ptr_ = msg;
        acu_sweep_ctrl_cmd_entity_.Reset();
        acu_sweep_ctrl_cmd_entity_.UpdateData(msg->fan_speed_ctrl, msg->mowing_speed_ctrl, msg->sweep_mode_ctrl, msg->fan_speed_mode, msg->fan_mode_ctrl, msg->sweep_plate_up_down, msg->mouthpiece_up_down_ctrl, msg->shaker_duster_ctrl, msg->dedusting_ctrl, msg->auto_garbage_dump_start_ctrl, msg->auto_cleaning_start_ctrl);
        can_msgs::msg::Frame acu_sweep_ctrl_cmd_can_msg;
        acu_sweep_ctrl_cmd_can_msg.header.stamp = msg->header.stamp;
        acu_sweep_ctrl_cmd_can_msg.dlc = 8;
        acu_sweep_ctrl_cmd_can_msg.id = acu_sweep_ctrl_cmd_entity_.ID;
        acu_sweep_ctrl_cmd_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = acu_sweep_ctrl_cmd_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            acu_sweep_ctrl_cmd_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        acu_sweep_ctrl_cmd_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(acu_sweep_ctrl_cmd_can_msg);
    }

    

void ControlCommand::callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_engage_ = msg->data;
}

void ControlCommand::timerCallback()
{
  if (!is_engage_) return;
  const rclcpp::Time current_time = this->now();

  // publishing msg
  /** example
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
  **/
  

    // throttle_command
    const double throttle_command_delta_time_ms =
        (current_time - throttle_command_received_time_).seconds() * 1000.0;
    if (throttle_command_delta_time_ms > param_.command_timeout_ms || throttle_command_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "throttle_command timeout = %f ms.", throttle_command_delta_time_ms);
    } else {
        can_frame_pub_->publish(*throttle_command_can_ptr_);
    }

    

    // brake_command
    const double brake_command_delta_time_ms =
        (current_time - brake_command_received_time_).seconds() * 1000.0;
    if (brake_command_delta_time_ms > param_.command_timeout_ms || brake_command_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "brake_command timeout = %f ms.", brake_command_delta_time_ms);
    } else {
        can_frame_pub_->publish(*brake_command_can_ptr_);
    }

    

    // steering_command
    const double steering_command_delta_time_ms =
        (current_time - steering_command_received_time_).seconds() * 1000.0;
    if (steering_command_delta_time_ms > param_.command_timeout_ms || steering_command_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "steering_command timeout = %f ms.", steering_command_delta_time_ms);
    } else {
        can_frame_pub_->publish(*steering_command_can_ptr_);
    }

    

    // gear_command
    const double gear_command_delta_time_ms =
        (current_time - gear_command_received_time_).seconds() * 1000.0;
    if (gear_command_delta_time_ms > param_.command_timeout_ms || gear_command_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "gear_command timeout = %f ms.", gear_command_delta_time_ms);
    } else {
        can_frame_pub_->publish(*gear_command_can_ptr_);
    }

    

    // park_command
    const double park_command_delta_time_ms =
        (current_time - park_command_received_time_).seconds() * 1000.0;
    if (park_command_delta_time_ms > param_.command_timeout_ms || park_command_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "park_command timeout = %f ms.", park_command_delta_time_ms);
    } else {
        can_frame_pub_->publish(*park_command_can_ptr_);
    }

    

    // vehicle_mode_command
    const double vehicle_mode_command_delta_time_ms =
        (current_time - vehicle_mode_command_received_time_).seconds() * 1000.0;
    if (vehicle_mode_command_delta_time_ms > param_.command_timeout_ms || vehicle_mode_command_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vehicle_mode_command timeout = %f ms.", vehicle_mode_command_delta_time_ms);
    } else {
        can_frame_pub_->publish(*vehicle_mode_command_can_ptr_);
    }

    

    // acu_sweep_ctrl_cmd
    const double acu_sweep_ctrl_cmd_delta_time_ms =
        (current_time - acu_sweep_ctrl_cmd_received_time_).seconds() * 1000.0;
    if (acu_sweep_ctrl_cmd_delta_time_ms > param_.command_timeout_ms || acu_sweep_ctrl_cmd_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "acu_sweep_ctrl_cmd timeout = %f ms.", acu_sweep_ctrl_cmd_delta_time_ms);
    } else {
        can_frame_pub_->publish(*acu_sweep_ctrl_cmd_can_ptr_);
    }

    
}

} // namespace control_command
} // namespace pix_sweeping_driver
