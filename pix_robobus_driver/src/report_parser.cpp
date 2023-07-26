#include <pix_robobus_driver/report_parser.hpp>

namespace pix_robobus_driver
{
namespace report_parser
{
ReportParser::ReportParser() : Node("report_parser")
{
  // ros params
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");
  param_.report_timeout_ms = declare_parameter("report_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);

  // // initialize msg received time
  /** example
  brake_command_received_time_ = this->now();
  **/
  chassis_sta_to_mobileye_received_time_ = this->now();
  auto_ctrl_msg_received_time_ = this->now();
  auto_remote_ctrl_msg_received_time_ = this->now();
  throttle_report_received_time_ = this->now();
  brake_report_received_time_ = this->now();
  steering_report_received_time_ = this->now();
  gear_report_received_time_ = this->now();
  park_report_received_time_ = this->now();
  vcu_report_received_time_ = this->now();
  wheel_speed_report_received_time_ = this->now();
  bms_report_received_time_ = this->now();
  vehicle_door_report_received_time_ = this->now();
  vehicle_mileage_fb_received_time_ = this->now();
  vcu_pad_transfer_received_time_ = this->now();


  is_publish_ = true;

  using std::placeholders::_1;

  /* subscriber */
  {
    // from pix driver autoware interface
    can_frame_sub_ = create_subscription<can_msgs::msg::Frame>(
      "input/can_rx", 1, std::bind(&ReportParser::callbackCan, this, _1));
    // is publish
    is_publish_sub_ = create_subscription<std_msgs::msg::Bool>(
      "input/is_publish", 1, std::bind(&ReportParser::callbackIsPublish, this, _1));
  }

  /* publisher */
  {
    /** example
    brake_sta_fb_pub_ =
      create_publisher<V2aBrakeStaFb>("/pix_robobus/v2a_brakestafb", rclcpp::QoS{1});
    **/
    chassis_sta_to_mobileye_pub_ = create_publisher<pix_robobus_driver_msgs::msg::ChassisStaToMobileye>("/pix_robobus/chassis_sta_to_mobileye", rclcpp::QoS{1});
    auto_ctrl_msg_pub_ = create_publisher<pix_robobus_driver_msgs::msg::AutoCtrlMsg>("/pix_robobus/auto_ctrl_msg", rclcpp::QoS{1});
    auto_remote_ctrl_msg_pub_ = create_publisher<pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg>("/pix_robobus/auto_remote_ctrl_msg", rclcpp::QoS{1});
    throttle_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::ThrottleReport>("/pix_robobus/throttle_report", rclcpp::QoS{1});
    brake_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::BrakeReport>("/pix_robobus/brake_report", rclcpp::QoS{1});
    steering_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::SteeringReport>("/pix_robobus/steering_report", rclcpp::QoS{1});
    gear_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::GearReport>("/pix_robobus/gear_report", rclcpp::QoS{1});
    park_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::ParkReport>("/pix_robobus/park_report", rclcpp::QoS{1});
    vcu_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::VcuReport>("/pix_robobus/vcu_report", rclcpp::QoS{1});
    wheel_speed_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::WheelSpeedReport>("/pix_robobus/wheel_speed_report", rclcpp::QoS{1});
    bms_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::BmsReport>("/pix_robobus/bms_report", rclcpp::QoS{1});
    vehicle_door_report_pub_ = create_publisher<pix_robobus_driver_msgs::msg::VehicleDoorReport>("/pix_robobus/vehicle_door_report", rclcpp::QoS{1});
    vehicle_mileage_fb_pub_ = create_publisher<pix_robobus_driver_msgs::msg::VehicleMileageFb>("/pix_robobus/vehicle_mileage_fb", rclcpp::QoS{1});
    vcu_pad_transfer_pub_ = create_publisher<std_msgs::msg::Bool>("/pix_robobus/vcu_pad_transfer", rclcpp::QoS{1});
 
  }
  {
    // timer
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
      std::bind(&ReportParser::timerCallback, this));
  }
}

// calback is publish
void ReportParser::callbackIsPublish(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_publish_ = msg->data;
}

// callback can
void ReportParser::callbackCan(const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  std_msgs::msg::Header header;
  header.frame_id = param_.base_frame_id;
  header.stamp = msg->header.stamp;

  /** example
  V2aBrakeStaFb brake_sta_fb_msg;
  **/
  pix_robobus_driver_msgs::msg::ChassisStaToMobileye chassis_sta_to_mobileye_msg;
  pix_robobus_driver_msgs::msg::AutoCtrlMsg auto_ctrl_msg_msg;
  pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg auto_remote_ctrl_msg_msg;
  pix_robobus_driver_msgs::msg::ThrottleReport throttle_report_msg;
  pix_robobus_driver_msgs::msg::BrakeReport brake_report_msg;
  pix_robobus_driver_msgs::msg::SteeringReport steering_report_msg;
  pix_robobus_driver_msgs::msg::GearReport gear_report_msg;
  pix_robobus_driver_msgs::msg::ParkReport park_report_msg;
  pix_robobus_driver_msgs::msg::VcuReport vcu_report_msg;
  pix_robobus_driver_msgs::msg::WheelSpeedReport wheel_speed_report_msg;
  pix_robobus_driver_msgs::msg::BmsReport bms_report_msg;
  pix_robobus_driver_msgs::msg::VehicleDoorReport vehicle_door_report_msg;
  pix_robobus_driver_msgs::msg::VehicleMileageFb vehicle_mileage_fb_msg;
  std_msgs::msg::Bool vcu_pad_transfer_msg;


  uint8_t byte_temp[8];
  switch (msg->id)
  {
  /** example
  case V2adrivestafb530::ID:
    drive_sta_fb_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_drivestafb_530_entity_.update_bytes(byte_temp);
    v2a_drivestafb_530_entity_.Parse();

    drive_sta_fb_msg.header = header;
    drive_sta_fb_msg.vcu_chassis_driver_en_sta =
      v2a_drivestafb_530_entity_.vcu_chassisdriverensta;
    drive_sta_fb_msg.vcu_chassis_diver_slopover =
      v2a_drivestafb_530_entity_.vcu_chassisdiverslopover;
    drive_sta_fb_msg.vcu_chassis_driver_mode_sta =
      v2a_drivestafb_530_entity_.vcu_chassisdrivermodesta;
    drive_sta_fb_msg.vcu_chassis_gear_fb = v2a_drivestafb_530_entity_.vcu_chassisgearfb;
    drive_sta_fb_msg.vcu_chassis_speed_fb = v2a_drivestafb_530_entity_.vcu_chassisspeedfb;
    drive_sta_fb_msg.vcu_chassis_throttle_padl_fb =
      v2a_drivestafb_530_entity_.vcu_chassisthrottlepaldfb;
    drive_sta_fb_msg.vcu_chassis_accceleration_fb =
      v2a_drivestafb_530_entity_.vcu_chassisaccelerationfb;
    drive_sta_fb_ptr_ = std::make_shared<V2aDriveStaFb>(drive_sta_fb_msg);
    break;
  **/
  

    case ChassisStaToMobileye::ID:
    chassis_sta_to_mobileye_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    chassis_sta_to_mobileye_entity_.update_bytes(byte_temp);
    chassis_sta_to_mobileye_entity_.Parse();

    chassis_sta_to_mobileye_msg.header = header;
    chassis_sta_to_mobileye_msg.vehicle_speed_to_mbe = chassis_sta_to_mobileye_entity_.vehicle_speed_to_mbe_;
chassis_sta_to_mobileye_msg.brake_light_actual = chassis_sta_to_mobileye_entity_.brake_light_actual_;
chassis_sta_to_mobileye_msg.turn_light_actual = chassis_sta_to_mobileye_entity_.turn_light_actual_;

    chassis_sta_to_mobileye_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::ChassisStaToMobileye>(chassis_sta_to_mobileye_msg);
    break;
    

    case AutoCtrlMsg::ID:
    auto_ctrl_msg_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    auto_ctrl_msg_entity_.update_bytes(byte_temp);
    auto_ctrl_msg_entity_.Parse();

    auto_ctrl_msg_msg.header = header;
    auto_ctrl_msg_msg.auto_heartbeat = auto_ctrl_msg_entity_.auto_heartbeat_;
auto_ctrl_msg_msg.auto_drive_ctrl_mode = auto_ctrl_msg_entity_.auto_drive_ctrl_mode_;

    auto_ctrl_msg_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::AutoCtrlMsg>(auto_ctrl_msg_msg);
    break;
    

    case AutoRemoteCtrlMsg::ID:
    auto_remote_ctrl_msg_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    auto_remote_ctrl_msg_entity_.update_bytes(byte_temp);
    auto_remote_ctrl_msg_entity_.Parse();

    auto_remote_ctrl_msg_msg.header = header;
    auto_remote_ctrl_msg_msg.auto_remote_heartbeat = auto_remote_ctrl_msg_entity_.auto_remote_heartbeat_;
auto_remote_ctrl_msg_msg.auto_remote_drive_ctrl_mode = auto_remote_ctrl_msg_entity_.auto_remote_drive_ctrl_mode_;

    auto_remote_ctrl_msg_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg>(auto_remote_ctrl_msg_msg);
    break;
    

    case ThrottleReport::ID:
    throttle_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    throttle_report_entity_.update_bytes(byte_temp);
    throttle_report_entity_.Parse();

    throttle_report_msg.header = header;
    throttle_report_msg.dirve_throttle_pedal_actual = throttle_report_entity_.dirve_throttle_pedal_actual_;
throttle_report_msg.dirve_flt2 = throttle_report_entity_.dirve_flt2_;
throttle_report_msg.dirve_flt1 = throttle_report_entity_.dirve_flt1_;
throttle_report_msg.dirve_en_state = throttle_report_entity_.dirve_en_state_;

    throttle_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::ThrottleReport>(throttle_report_msg);
    break;
    

    case BrakeReport::ID:
    brake_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    brake_report_entity_.update_bytes(byte_temp);
    brake_report_entity_.Parse();

    brake_report_msg.header = header;
    brake_report_msg.brake_pedal_actual = brake_report_entity_.brake_pedal_actual_;
brake_report_msg.brake_flt2 = brake_report_entity_.brake_flt2_;
brake_report_msg.brake_flt1 = brake_report_entity_.brake_flt1_;
brake_report_msg.brake_en_state = brake_report_entity_.brake_en_state_;

    brake_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::BrakeReport>(brake_report_msg);
    break;
    

    case SteeringReport::ID:
    steering_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    steering_report_entity_.update_bytes(byte_temp);
    steering_report_entity_.Parse();

    steering_report_msg.header = header;
    steering_report_msg.steer_angle_speed_set_val = steering_report_entity_.steer_angle_speed_set_val_;
steering_report_msg.steer_flt2 = steering_report_entity_.steer_flt2_;
steering_report_msg.steer_flt1 = steering_report_entity_.steer_flt1_;
steering_report_msg.steer_en_state = steering_report_entity_.steer_en_state_;
steering_report_msg.steer_angle_actual = steering_report_entity_.steer_angle_actual_;

    steering_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::SteeringReport>(steering_report_msg);
    break;
    

    case GearReport::ID:
    gear_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    gear_report_entity_.update_bytes(byte_temp);
    gear_report_entity_.Parse();

    gear_report_msg.header = header;
    gear_report_msg.gear_flt = gear_report_entity_.gear_flt_;
gear_report_msg.gear_actual = gear_report_entity_.gear_actual_;

    gear_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::GearReport>(gear_report_msg);
    break;
    

    case ParkReport::ID:
    park_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    park_report_entity_.update_bytes(byte_temp);
    park_report_entity_.Parse();

    park_report_msg.header = header;
    park_report_msg.parking_actual = park_report_entity_.parking_actual_;
park_report_msg.park_flt = park_report_entity_.park_flt_;

    park_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::ParkReport>(park_report_msg);
    break;
    

    case VcuReport::ID:
    vcu_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu_report_entity_.update_bytes(byte_temp);
    vcu_report_entity_.Parse();

    vcu_report_msg.header = header;
    vcu_report_msg.allow_self_driving_run = vcu_report_entity_.allow_self_driving_run_;
vcu_report_msg.vcu_chassis_estop_sta_fb = vcu_report_entity_.vcu_chassis_estop_sta_fb_;
vcu_report_msg.clearance_lamp_actual = vcu_report_entity_.clearance_lamp_actual_;
vcu_report_msg.car_work_state = vcu_report_entity_.car_work_state_;
vcu_report_msg.car_power_state = vcu_report_entity_.car_power_state_;
vcu_report_msg.aeb_trigger_state = vcu_report_entity_.aeb_trigger_state_;
vcu_report_msg.brake_light_actual = vcu_report_entity_.brake_light_actual_;
vcu_report_msg.headlight_actual = vcu_report_entity_.headlight_actual_;
vcu_report_msg.turn_light_actual = vcu_report_entity_.turn_light_actual_;
vcu_report_msg.vehicle_errcode = vcu_report_entity_.vehicle_errcode_;
vcu_report_msg.drive_mode_status = vcu_report_entity_.drive_mode_status_;
vcu_report_msg.steer_mode_status = vcu_report_entity_.steer_mode_status_;
vcu_report_msg.vehicle_mode_state = vcu_report_entity_.vehicle_mode_state_;
vcu_report_msg.vehicle_front_crash_state = vcu_report_entity_.vehicle_front_crash_state_;
vcu_report_msg.back_crash_state = vcu_report_entity_.back_crash_state_;
vcu_report_msg.aeb_state = vcu_report_entity_.aeb_state_;
vcu_report_msg.vehicle_acc = vcu_report_entity_.vehicle_acc_;
vcu_report_msg.vehicle_speed = vcu_report_entity_.vehicle_speed_;

    vcu_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::VcuReport>(vcu_report_msg);
    break;
    

    case WheelSpeedReport::ID:
    wheel_speed_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    wheel_speed_report_entity_.update_bytes(byte_temp);
    wheel_speed_report_entity_.Parse();

    wheel_speed_report_msg.header = header;
    wheel_speed_report_msg.wheel_speed_rr = wheel_speed_report_entity_.wheel_speed_rr_;
wheel_speed_report_msg.wheel_speed_rl = wheel_speed_report_entity_.wheel_speed_rl_;
wheel_speed_report_msg.wheel_speed_fr = wheel_speed_report_entity_.wheel_speed_fr_;
wheel_speed_report_msg.wheel_speed_fl = wheel_speed_report_entity_.wheel_speed_fl_;

    wheel_speed_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::WheelSpeedReport>(wheel_speed_report_msg);
    break;
    

    case BmsReport::ID:
    bms_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    bms_report_entity_.update_bytes(byte_temp);
    bms_report_entity_.Parse();

    bms_report_msg.header = header;
    bms_report_msg.battery_leadacid_voltage = bms_report_entity_.battery_leadacid_voltage_;
bms_report_msg.battery_current = bms_report_entity_.battery_current_;
bms_report_msg.battery_voltage = bms_report_entity_.battery_voltage_;
bms_report_msg.battery_soc = bms_report_entity_.battery_soc_;

    bms_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::BmsReport>(bms_report_msg);
    break;
    

    case VehicleDoorReport::ID:
    vehicle_door_report_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vehicle_door_report_entity_.update_bytes(byte_temp);
    vehicle_door_report_entity_.Parse();

    vehicle_door_report_msg.header = header;
    vehicle_door_report_msg.door_report_sta = vehicle_door_report_entity_.door_report_sta_;
vehicle_door_report_msg.door_open_timeout = vehicle_door_report_entity_.door_open_timeout_;
vehicle_door_report_msg.door_open_sta = vehicle_door_report_entity_.door_open_sta_;
vehicle_door_report_msg.door_open_inplace = vehicle_door_report_entity_.door_open_inplace_;
vehicle_door_report_msg.door_close_timeout = vehicle_door_report_entity_.door_close_timeout_;
vehicle_door_report_msg.door_close_sta = vehicle_door_report_entity_.door_close_sta_;
vehicle_door_report_msg.door_close_inplace = vehicle_door_report_entity_.door_close_inplace_;
vehicle_door_report_msg.door_button_enable = vehicle_door_report_entity_.door_button_enable_;

    vehicle_door_report_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::VehicleDoorReport>(vehicle_door_report_msg);
    break;
    

    case VehicleMileageFb::ID:
      vehicle_mileage_fb_received_time_ = this->now();
      
      for(uint i=0;i<8;i++)
      {
        byte_temp[i] = msg->data[i];
      }
      vehicle_mileage_fb_entity_.update_bytes(byte_temp);
      vehicle_mileage_fb_entity_.Parse();

      vehicle_mileage_fb_msg.header = header;
      vehicle_mileage_fb_msg.vehicle_sub_mileage = vehicle_mileage_fb_entity_.vehicle_sub_mileage_;
      vehicle_mileage_fb_msg.vehicle_odo = vehicle_mileage_fb_entity_.vehicle_odo_;

      vehicle_mileage_fb_ptr_ = std::make_shared<pix_robobus_driver_msgs::msg::VehicleMileageFb>(vehicle_mileage_fb_msg);
      break;
    
    case VCUPadTransfer::ID:
      vcu_pad_transfer_received_time_ = this->now();
      for(uint i=0;i<8;i++)
      {
        byte_temp[i] = msg->data[i];
      }
      vcu_pad_transfer_entity_.update_bytes(byte_temp);
      vcu_pad_transfer_entity_.Parse();
      vcu_pad_transfer_msg.data = vcu_pad_transfer_entity_.v_c_u__pad_auto_start_;
      vcu_pad_transfer_ptr_ = std::make_shared<std_msgs::msg::Bool>(vcu_pad_transfer_msg);
      break;


    default:
    break;
  }
}

void ReportParser::timerCallback()
{
  if (!is_publish_) return;

  const rclcpp::Time current_time = this->now();
  
  /** example
  // drive sta fb report
  const double drive_sta_fb_report_delta_time_ms =
    (current_time - drive_sta_fb_received_time_).seconds() * 1000.0;
  if(drive_sta_fb_report_delta_time_ms>param_.report_timeout_ms || drive_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "drive stat fb report timeout = %f ms.", drive_sta_fb_report_delta_time_ms);
  }else{
    drive_sta_fb_pub_->publish(*drive_sta_fb_ptr_);
  }
  **/
  
    const double chassis_sta_to_mobileye_report_delta_time_ms =
    (current_time - chassis_sta_to_mobileye_received_time_).seconds() * 1000.0;
    if(chassis_sta_to_mobileye_report_delta_time_ms>param_.report_timeout_ms || chassis_sta_to_mobileye_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "chassis_sta_to_mobileye report timeout = %f ms.", chassis_sta_to_mobileye_report_delta_time_ms);
    }else{
        chassis_sta_to_mobileye_pub_->publish(*chassis_sta_to_mobileye_ptr_);
    }
    
    const double auto_ctrl_msg_report_delta_time_ms =
    (current_time - auto_ctrl_msg_received_time_).seconds() * 1000.0;
    if(auto_ctrl_msg_report_delta_time_ms>param_.report_timeout_ms || auto_ctrl_msg_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "auto_ctrl_msg report timeout = %f ms.", auto_ctrl_msg_report_delta_time_ms);
    }else{
        auto_ctrl_msg_pub_->publish(*auto_ctrl_msg_ptr_);
    }
    
    const double auto_remote_ctrl_msg_report_delta_time_ms =
    (current_time - auto_remote_ctrl_msg_received_time_).seconds() * 1000.0;
    if(auto_remote_ctrl_msg_report_delta_time_ms>param_.report_timeout_ms || auto_remote_ctrl_msg_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "auto_remote_ctrl_msg report timeout = %f ms.", auto_remote_ctrl_msg_report_delta_time_ms);
    }else{
        auto_remote_ctrl_msg_pub_->publish(*auto_remote_ctrl_msg_ptr_);
    }
    
    const double throttle_report_report_delta_time_ms =
    (current_time - throttle_report_received_time_).seconds() * 1000.0;
    if(throttle_report_report_delta_time_ms>param_.report_timeout_ms || throttle_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "throttle_report report timeout = %f ms.", throttle_report_report_delta_time_ms);
    }else{
        throttle_report_pub_->publish(*throttle_report_ptr_);
    }
    
    const double brake_report_report_delta_time_ms =
    (current_time - brake_report_received_time_).seconds() * 1000.0;
    if(brake_report_report_delta_time_ms>param_.report_timeout_ms || brake_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "brake_report report timeout = %f ms.", brake_report_report_delta_time_ms);
    }else{
        brake_report_pub_->publish(*brake_report_ptr_);
    }
    
    const double steering_report_report_delta_time_ms =
    (current_time - steering_report_received_time_).seconds() * 1000.0;
    if(steering_report_report_delta_time_ms>param_.report_timeout_ms || steering_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "steering_report report timeout = %f ms.", steering_report_report_delta_time_ms);
    }else{
        steering_report_pub_->publish(*steering_report_ptr_);
    }
    
    const double gear_report_report_delta_time_ms =
    (current_time - gear_report_received_time_).seconds() * 1000.0;
    if(gear_report_report_delta_time_ms>param_.report_timeout_ms || gear_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "gear_report report timeout = %f ms.", gear_report_report_delta_time_ms);
    }else{
        gear_report_pub_->publish(*gear_report_ptr_);
    }
    
    const double park_report_report_delta_time_ms =
    (current_time - park_report_received_time_).seconds() * 1000.0;
    if(park_report_report_delta_time_ms>param_.report_timeout_ms || park_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "park_report report timeout = %f ms.", park_report_report_delta_time_ms);
    }else{
        park_report_pub_->publish(*park_report_ptr_);
    }
    
    const double vcu_report_report_delta_time_ms =
    (current_time - vcu_report_received_time_).seconds() * 1000.0;
    if(vcu_report_report_delta_time_ms>param_.report_timeout_ms || vcu_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu_report report timeout = %f ms.", vcu_report_report_delta_time_ms);
    }else{
        vcu_report_pub_->publish(*vcu_report_ptr_);
    }
    
    const double wheel_speed_report_report_delta_time_ms =
    (current_time - wheel_speed_report_received_time_).seconds() * 1000.0;
    if(wheel_speed_report_report_delta_time_ms>param_.report_timeout_ms || wheel_speed_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "wheel_speed_report report timeout = %f ms.", wheel_speed_report_report_delta_time_ms);
    }else{
        wheel_speed_report_pub_->publish(*wheel_speed_report_ptr_);
    }
    
    const double bms_report_report_delta_time_ms =
    (current_time - bms_report_received_time_).seconds() * 1000.0;
    if(bms_report_report_delta_time_ms>param_.report_timeout_ms || bms_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "bms_report report timeout = %f ms.", bms_report_report_delta_time_ms);
    }else{
        bms_report_pub_->publish(*bms_report_ptr_);
    }
    
    const double vehicle_door_report_report_delta_time_ms =
    (current_time - vehicle_door_report_received_time_).seconds() * 1000.0;
    if(vehicle_door_report_report_delta_time_ms>param_.report_timeout_ms || vehicle_door_report_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vehicle_door_report report timeout = %f ms.", vehicle_door_report_report_delta_time_ms);
    }else{
        vehicle_door_report_pub_->publish(*vehicle_door_report_ptr_);
    }
    
    const double vehicle_mileage_fb_report_delta_time_ms =
    (current_time - vehicle_mileage_fb_received_time_).seconds() * 1000.0;
    if(vehicle_mileage_fb_report_delta_time_ms>param_.report_timeout_ms || vehicle_mileage_fb_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vehicle_mileage_fb report timeout = %f ms.", vehicle_mileage_fb_report_delta_time_ms);
    }else{
        vehicle_mileage_fb_pub_->publish(*vehicle_mileage_fb_ptr_);
    }
    
    const double vcu_pad_transfer_report_delta_time_ms =
    (current_time - vcu_pad_transfer_received_time_).seconds() * 1000.0;
    if(vcu_pad_transfer_report_delta_time_ms>param_.report_timeout_ms || vcu_pad_transfer_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu_pad_transfer report timeout = %f ms.", vehicle_mileage_fb_report_delta_time_ms);
    }else{
        vcu_pad_transfer_pub_->publish(*vcu_pad_transfer_ptr_);
    }
}

} // namespace report_parser
} // namespace pix_robobus_driver
