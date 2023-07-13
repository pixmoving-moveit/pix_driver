#include <pix_sweeping_driver/report_parser.hpp>

namespace pix_sweeping_driver
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
  vcu2_acu_chassis_err_code1_received_time_ = this->now();
vcu2_acu_chassis_err_code2_received_time_ = this->now();
vcu2_acu_chassis_err_code3_received_time_ = this->now();
throttle_report_received_time_ = this->now();
brake_report_received_time_ = this->now();
steering_report_received_time_ = this->now();
gear_report_received_time_ = this->now();
park_report_received_time_ = this->now();
vcu_report_received_time_ = this->now();
wheel_speed_report_received_time_ = this->now();
bms_report_received_time_ = this->now();
vcu2_acu_sweep_sta_fb_received_time_ = this->now();
vcu2_acu_sweep_fan_sta_received_time_ = this->now();
vcu2_acu_sweep_sta_received_time_ = this->now();
scu_work_time_fb_received_time_ = this->now();
vcu2_acu_sweep_work_sta_received_time_ = this->now();
vehicle_mileage_fb_received_time_ = this->now();


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
      create_publisher<V2aBrakeStaFb>("/pix_hooke/v2a_brakestafb", rclcpp::QoS{1});
    **/
    vcu2_acu_chassis_err_code1_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode1>("/pix_sweeping/vcu2_acu_chassis_err_code1", rclcpp::QoS{1});
    vcu2_acu_chassis_err_code2_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode2>("/pix_sweeping/vcu2_acu_chassis_err_code2", rclcpp::QoS{1});
    vcu2_acu_chassis_err_code3_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode3>("/pix_sweeping/vcu2_acu_chassis_err_code3", rclcpp::QoS{1});
    throttle_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::ThrottleReport>("/pix_sweeping/throttle_report", rclcpp::QoS{1});
    brake_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::BrakeReport>("/pix_sweeping/brake_report", rclcpp::QoS{1});
    steering_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::SteeringReport>("/pix_sweeping/steering_report", rclcpp::QoS{1});
    gear_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::GearReport>("/pix_sweeping/gear_report", rclcpp::QoS{1});
    park_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::ParkReport>("/pix_sweeping/park_report", rclcpp::QoS{1});
    vcu_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::VcuReport>("/pix_sweeping/vcu_report", rclcpp::QoS{1});
    wheel_speed_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::WheelSpeedReport>("/pix_sweeping/wheel_speed_report", rclcpp::QoS{1});
    bms_report_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::BmsReport>("/pix_sweeping/bms_report", rclcpp::QoS{1});
    vcu2_acu_sweep_sta_fb_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepStaFb>("/pix_sweeping/vcu2_acu_sweep_sta_fb", rclcpp::QoS{1});
    vcu2_acu_sweep_fan_sta_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepFanSta>("/pix_sweeping/vcu2_acu_sweep_fan_sta", rclcpp::QoS{1});
    vcu2_acu_sweep_sta_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepSta>("/pix_sweeping/vcu2_acu_sweep_sta", rclcpp::QoS{1});
    scu_work_time_fb_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::ScuWorkTimeFb>("/pix_sweeping/scu_work_time_fb", rclcpp::QoS{1});
    vcu2_acu_sweep_work_sta_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepWorkSta>("/pix_sweeping/vcu2_acu_sweep_work_sta", rclcpp::QoS{1});
    vehicle_mileage_fb_pub_ = create_publisher<pix_sweeping_driver_msgs::msg::VehicleMileageFb>("/pix_sweeping/vehicle_mileage_fb", rclcpp::QoS{1});
 
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
  pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode1 vcu2_acu_chassis_err_code1_msg;
pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode2 vcu2_acu_chassis_err_code2_msg;
pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode3 vcu2_acu_chassis_err_code3_msg;
pix_sweeping_driver_msgs::msg::ThrottleReport throttle_report_msg;
pix_sweeping_driver_msgs::msg::BrakeReport brake_report_msg;
pix_sweeping_driver_msgs::msg::SteeringReport steering_report_msg;
pix_sweeping_driver_msgs::msg::GearReport gear_report_msg;
pix_sweeping_driver_msgs::msg::ParkReport park_report_msg;
pix_sweeping_driver_msgs::msg::VcuReport vcu_report_msg;
pix_sweeping_driver_msgs::msg::WheelSpeedReport wheel_speed_report_msg;
pix_sweeping_driver_msgs::msg::BmsReport bms_report_msg;
pix_sweeping_driver_msgs::msg::Vcu2AcuSweepStaFb vcu2_acu_sweep_sta_fb_msg;
pix_sweeping_driver_msgs::msg::Vcu2AcuSweepFanSta vcu2_acu_sweep_fan_sta_msg;
pix_sweeping_driver_msgs::msg::Vcu2AcuSweepSta vcu2_acu_sweep_sta_msg;
pix_sweeping_driver_msgs::msg::ScuWorkTimeFb scu_work_time_fb_msg;
pix_sweeping_driver_msgs::msg::Vcu2AcuSweepWorkSta vcu2_acu_sweep_work_sta_msg;
pix_sweeping_driver_msgs::msg::VehicleMileageFb vehicle_mileage_fb_msg;


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
  

    case Vcu2AcuChassisErrCode1::ID:
    vcu2_acu_chassis_err_code1_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu2_acu_chassis_err_code1_entity_.update_bytes(byte_temp);
    vcu2_acu_chassis_err_code1_entity_.Parse();

    vcu2_acu_chassis_err_code1_msg.header = header;
    vcu2_acu_chassis_err_code1_msg.bms_communication_fault = vcu2_acu_chassis_err_code1_entity_.bms_communication_fault_;
vcu2_acu_chassis_err_code1_msg.chassis_emergency_sta = vcu2_acu_chassis_err_code1_entity_.chassis_emergency_sta_;
vcu2_acu_chassis_err_code1_msg.leadacid_battery_low = vcu2_acu_chassis_err_code1_entity_.leadacid_battery_low_;
vcu2_acu_chassis_err_code1_msg.controler_communication_fault = vcu2_acu_chassis_err_code1_entity_.controler_communication_fault_;
vcu2_acu_chassis_err_code1_msg.chassis_crash_sta = vcu2_acu_chassis_err_code1_entity_.chassis_crash_sta_;
vcu2_acu_chassis_err_code1_msg.eps_rear_communication_fault = vcu2_acu_chassis_err_code1_entity_.eps_rear_communication_fault_;
vcu2_acu_chassis_err_code1_msg.eps_front_communication_fault = vcu2_acu_chassis_err_code1_entity_.eps_front_communication_fault_;
vcu2_acu_chassis_err_code1_msg.eds_rr_communication_fault = vcu2_acu_chassis_err_code1_entity_.eds_rr_communication_fault_;
vcu2_acu_chassis_err_code1_msg.eds_rf_communication_fault = vcu2_acu_chassis_err_code1_entity_.eds_rf_communication_fault_;
vcu2_acu_chassis_err_code1_msg.eds_lr_communication_fault = vcu2_acu_chassis_err_code1_entity_.eds_lr_communication_fault_;
vcu2_acu_chassis_err_code1_msg.eds_lf_communication_fault = vcu2_acu_chassis_err_code1_entity_.eds_lf_communication_fault_;
vcu2_acu_chassis_err_code1_msg.ebs_communication_fault = vcu2_acu_chassis_err_code1_entity_.ebs_communication_fault_;
vcu2_acu_chassis_err_code1_msg.chassis_vcu_other_err = vcu2_acu_chassis_err_code1_entity_.chassis_vcu_other_err_;
vcu2_acu_chassis_err_code1_msg.bms_fault1 = vcu2_acu_chassis_err_code1_entity_.bms_fault1_;
vcu2_acu_chassis_err_code1_msg.bms_fault2 = vcu2_acu_chassis_err_code1_entity_.bms_fault2_;
vcu2_acu_chassis_err_code1_msg.bms_fault3 = vcu2_acu_chassis_err_code1_entity_.bms_fault3_;
vcu2_acu_chassis_err_code1_msg.bms_fault4 = vcu2_acu_chassis_err_code1_entity_.bms_fault4_;
vcu2_acu_chassis_err_code1_msg.bms_fault5 = vcu2_acu_chassis_err_code1_entity_.bms_fault5_;
vcu2_acu_chassis_err_code1_msg.bms_fault6 = vcu2_acu_chassis_err_code1_entity_.bms_fault6_;
vcu2_acu_chassis_err_code1_msg.bms_fault7 = vcu2_acu_chassis_err_code1_entity_.bms_fault7_;
vcu2_acu_chassis_err_code1_msg.bms_fault8 = vcu2_acu_chassis_err_code1_entity_.bms_fault8_;
vcu2_acu_chassis_err_code1_msg.bms_fault9 = vcu2_acu_chassis_err_code1_entity_.bms_fault9_;
vcu2_acu_chassis_err_code1_msg.bms_fault10 = vcu2_acu_chassis_err_code1_entity_.bms_fault10_;
vcu2_acu_chassis_err_code1_msg.bms_fault11 = vcu2_acu_chassis_err_code1_entity_.bms_fault11_;
vcu2_acu_chassis_err_code1_msg.bms_fault12 = vcu2_acu_chassis_err_code1_entity_.bms_fault12_;
vcu2_acu_chassis_err_code1_msg.bms_fault13 = vcu2_acu_chassis_err_code1_entity_.bms_fault13_;

    vcu2_acu_chassis_err_code1_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode1>(vcu2_acu_chassis_err_code1_msg);
    break;
    

    case Vcu2AcuChassisErrCode2::ID:
    vcu2_acu_chassis_err_code2_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu2_acu_chassis_err_code2_entity_.update_bytes(byte_temp);
    vcu2_acu_chassis_err_code2_entity_.Parse();

    vcu2_acu_chassis_err_code2_msg.header = header;
    vcu2_acu_chassis_err_code2_msg.chassis_ebs_error_code3 = vcu2_acu_chassis_err_code2_entity_.chassis_ebs_error_code3_;
vcu2_acu_chassis_err_code2_msg.chassis_bms_error_code = vcu2_acu_chassis_err_code2_entity_.chassis_bms_error_code_;
vcu2_acu_chassis_err_code2_msg.chassis_back_eps_error_code = vcu2_acu_chassis_err_code2_entity_.chassis_back_eps_error_code_;
vcu2_acu_chassis_err_code2_msg.chassis_front_eps_error_code = vcu2_acu_chassis_err_code2_entity_.chassis_front_eps_error_code_;
vcu2_acu_chassis_err_code2_msg.chassis_ebs_error_code2 = vcu2_acu_chassis_err_code2_entity_.chassis_ebs_error_code2_;
vcu2_acu_chassis_err_code2_msg.chassis_ebs_error_code1 = vcu2_acu_chassis_err_code2_entity_.chassis_ebs_error_code1_;
vcu2_acu_chassis_err_code2_msg.chassis_ebs_errorlv = vcu2_acu_chassis_err_code2_entity_.chassis_ebs_errorlv_;

    vcu2_acu_chassis_err_code2_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode2>(vcu2_acu_chassis_err_code2_msg);
    break;
    

    case Vcu2AcuChassisErrCode3::ID:
    vcu2_acu_chassis_err_code3_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu2_acu_chassis_err_code3_entity_.update_bytes(byte_temp);
    vcu2_acu_chassis_err_code3_entity_.Parse();

    vcu2_acu_chassis_err_code3_msg.header = header;
    vcu2_acu_chassis_err_code3_msg.chassis_rr_eds_error_code2 = vcu2_acu_chassis_err_code3_entity_.chassis_rr_eds_error_code2_;
vcu2_acu_chassis_err_code3_msg.chassis_rf_eds_error_code2 = vcu2_acu_chassis_err_code3_entity_.chassis_rf_eds_error_code2_;
vcu2_acu_chassis_err_code3_msg.chassis_lr_eds_error_code2 = vcu2_acu_chassis_err_code3_entity_.chassis_lr_eds_error_code2_;
vcu2_acu_chassis_err_code3_msg.chassis_lf_eds_error_code2 = vcu2_acu_chassis_err_code3_entity_.chassis_lf_eds_error_code2_;
vcu2_acu_chassis_err_code3_msg.chassis_rr_eds_error_code = vcu2_acu_chassis_err_code3_entity_.chassis_rr_eds_error_code_;
vcu2_acu_chassis_err_code3_msg.chassis_rf_eds_error_code = vcu2_acu_chassis_err_code3_entity_.chassis_rf_eds_error_code_;
vcu2_acu_chassis_err_code3_msg.chassis_lr_eds_error_code = vcu2_acu_chassis_err_code3_entity_.chassis_lr_eds_error_code_;
vcu2_acu_chassis_err_code3_msg.chassis_lf_eds_error_code = vcu2_acu_chassis_err_code3_entity_.chassis_lf_eds_error_code_;

    vcu2_acu_chassis_err_code3_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::Vcu2AcuChassisErrCode3>(vcu2_acu_chassis_err_code3_msg);
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

    throttle_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::ThrottleReport>(throttle_report_msg);
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

    brake_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::BrakeReport>(brake_report_msg);
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
    steering_report_msg.steer_angle_speed_actual = steering_report_entity_.steer_angle_speed_actual_;
steering_report_msg.steer_angle_rear_actual = steering_report_entity_.steer_angle_rear_actual_;
steering_report_msg.steer_angle_actual = steering_report_entity_.steer_angle_actual_;
steering_report_msg.steer_flt2 = steering_report_entity_.steer_flt2_;
steering_report_msg.steer_flt1 = steering_report_entity_.steer_flt1_;
steering_report_msg.steer_en_state = steering_report_entity_.steer_en_state_;

    steering_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::SteeringReport>(steering_report_msg);
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

    gear_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::GearReport>(gear_report_msg);
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
    park_report_msg.park_flt = park_report_entity_.park_flt_;
park_report_msg.parking_actual = park_report_entity_.parking_actual_;

    park_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::ParkReport>(park_report_msg);
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
    vcu_report_msg.turn_light_actual = vcu_report_entity_.turn_light_actual_;
vcu_report_msg.aeb_trigger_state = vcu_report_entity_.aeb_trigger_state_;
vcu_report_msg.headlight_actual = vcu_report_entity_.headlight_actual_;
vcu_report_msg.car_work_state = vcu_report_entity_.car_work_state_;
vcu_report_msg.car_power_state = vcu_report_entity_.car_power_state_;
vcu_report_msg.vehicle_errcode = vcu_report_entity_.vehicle_errcode_;
vcu_report_msg.aeb_brake_state = vcu_report_entity_.aeb_brake_state_;
vcu_report_msg.front_crash_state = vcu_report_entity_.front_crash_state_;
vcu_report_msg.back_crash_state = vcu_report_entity_.back_crash_state_;
vcu_report_msg.vehicle_mode_state = vcu_report_entity_.vehicle_mode_state_;
vcu_report_msg.drive_mode_status = vcu_report_entity_.drive_mode_status_;
vcu_report_msg.vehicle_speed = vcu_report_entity_.vehicle_speed_;
vcu_report_msg.steer_mode_status = vcu_report_entity_.steer_mode_status_;
vcu_report_msg.brake_light_actual = vcu_report_entity_.brake_light_actual_;
vcu_report_msg.vehicle_acc = vcu_report_entity_.vehicle_acc_;

    vcu_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::VcuReport>(vcu_report_msg);
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
    wheel_speed_report_msg.wheel_speed_r_r = wheel_speed_report_entity_.wheel_speed_r_r_;
wheel_speed_report_msg.wheel_speed_r_l = wheel_speed_report_entity_.wheel_speed_r_l_;
wheel_speed_report_msg.wheel_speed_f_r = wheel_speed_report_entity_.wheel_speed_f_r_;
wheel_speed_report_msg.wheel_speed_f_l = wheel_speed_report_entity_.wheel_speed_f_l_;

    wheel_speed_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::WheelSpeedReport>(wheel_speed_report_msg);
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
bms_report_msg.wireless_charging_sta = bms_report_entity_.wireless_charging_sta_;
bms_report_msg.battery_soc = bms_report_entity_.battery_soc_;
bms_report_msg.battery_current = bms_report_entity_.battery_current_;
bms_report_msg.battery_voltage = bms_report_entity_.battery_voltage_;

    bms_report_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::BmsReport>(bms_report_msg);
    break;
    

    case Vcu2AcuSweepStaFb::ID:
    vcu2_acu_sweep_sta_fb_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu2_acu_sweep_sta_fb_entity_.update_bytes(byte_temp);
    vcu2_acu_sweep_sta_fb_entity_.Parse();

    vcu2_acu_sweep_sta_fb_msg.header = header;
    vcu2_acu_sweep_sta_fb_msg.scu_mowing_speed_fb = vcu2_acu_sweep_sta_fb_entity_.scu_mowing_speed_fb_;
vcu2_acu_sweep_sta_fb_msg.push_rod_travel_difference_alarm = vcu2_acu_sweep_sta_fb_entity_.push_rod_travel_difference_alarm_;
vcu2_acu_sweep_sta_fb_msg.end_of_garbage_dumping_des = vcu2_acu_sweep_sta_fb_entity_.end_of_garbage_dumping_des_;
vcu2_acu_sweep_sta_fb_msg.end_of_garbage_dumping = vcu2_acu_sweep_sta_fb_entity_.end_of_garbage_dumping_;
vcu2_acu_sweep_sta_fb_msg.scu_fan_speed_fb = vcu2_acu_sweep_sta_fb_entity_.scu_fan_speed_fb_;
vcu2_acu_sweep_sta_fb_msg.scu_liquid_leve = vcu2_acu_sweep_sta_fb_entity_.scu_liquid_leve_;
vcu2_acu_sweep_sta_fb_msg.scu_heartbeat = vcu2_acu_sweep_sta_fb_entity_.scu_heartbeat_;
vcu2_acu_sweep_sta_fb_msg.scan_controller_communication_fault = vcu2_acu_sweep_sta_fb_entity_.scan_controller_communication_fault_;
vcu2_acu_sweep_sta_fb_msg.fan_controller_communication_fault = vcu2_acu_sweep_sta_fb_entity_.fan_controller_communication_fault_;
vcu2_acu_sweep_sta_fb_msg.scu_filter_clogging = vcu2_acu_sweep_sta_fb_entity_.scu_filter_clogging_;
vcu2_acu_sweep_sta_fb_msg.scu_sweep_life_end = vcu2_acu_sweep_sta_fb_entity_.scu_sweep_life_end_;
vcu2_acu_sweep_sta_fb_msg.scu_dustbin_full = vcu2_acu_sweep_sta_fb_entity_.scu_dustbin_full_;
vcu2_acu_sweep_sta_fb_msg.scu_liquid_leve_high = vcu2_acu_sweep_sta_fb_entity_.scu_liquid_leve_high_;
vcu2_acu_sweep_sta_fb_msg.scu_liquid_level_low = vcu2_acu_sweep_sta_fb_entity_.scu_liquid_level_low_;

    vcu2_acu_sweep_sta_fb_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepStaFb>(vcu2_acu_sweep_sta_fb_msg);
    break;
    

    case Vcu2AcuSweepFanSta::ID:
    vcu2_acu_sweep_fan_sta_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu2_acu_sweep_fan_sta_entity_.update_bytes(byte_temp);
    vcu2_acu_sweep_fan_sta_entity_.Parse();

    vcu2_acu_sweep_fan_sta_msg.header = header;
    vcu2_acu_sweep_fan_sta_msg.scu_fan_conntorller_err2 = vcu2_acu_sweep_fan_sta_entity_.scu_fan_conntorller_err2_;
vcu2_acu_sweep_fan_sta_msg.scu_fan_conntorller_err1 = vcu2_acu_sweep_fan_sta_entity_.scu_fan_conntorller_err1_;
vcu2_acu_sweep_fan_sta_msg.scu_fan_conntorller_temp = vcu2_acu_sweep_fan_sta_entity_.scu_fan_conntorller_temp_;
vcu2_acu_sweep_fan_sta_msg.scu_fan_motor_temp = vcu2_acu_sweep_fan_sta_entity_.scu_fan_motor_temp_;
vcu2_acu_sweep_fan_sta_msg.scu_fan_conntorller_current = vcu2_acu_sweep_fan_sta_entity_.scu_fan_conntorller_current_;
vcu2_acu_sweep_fan_sta_msg.scu_fan_conntorller_main_voltage = vcu2_acu_sweep_fan_sta_entity_.scu_fan_conntorller_main_voltage_;

    vcu2_acu_sweep_fan_sta_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepFanSta>(vcu2_acu_sweep_fan_sta_msg);
    break;
    

    case Vcu2AcuSweepSta::ID:
    vcu2_acu_sweep_sta_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu2_acu_sweep_sta_entity_.update_bytes(byte_temp);
    vcu2_acu_sweep_sta_entity_.Parse();

    vcu2_acu_sweep_sta_msg.header = header;
    vcu2_acu_sweep_sta_msg.scu_sweep_conntorller_err2 = vcu2_acu_sweep_sta_entity_.scu_sweep_conntorller_err2_;
vcu2_acu_sweep_sta_msg.scu_sweep_conntorller_err1 = vcu2_acu_sweep_sta_entity_.scu_sweep_conntorller_err1_;
vcu2_acu_sweep_sta_msg.scu_sweep_conntroller_current = vcu2_acu_sweep_sta_entity_.scu_sweep_conntroller_current_;
vcu2_acu_sweep_sta_msg.scu_sweep_conntroller_voltage = vcu2_acu_sweep_sta_entity_.scu_sweep_conntroller_voltage_;
vcu2_acu_sweep_sta_msg.scu_sweep_speed = vcu2_acu_sweep_sta_entity_.scu_sweep_speed_;
vcu2_acu_sweep_sta_msg.scu_sweep_travel_mm = vcu2_acu_sweep_sta_entity_.scu_sweep_travel_mm_;

    vcu2_acu_sweep_sta_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepSta>(vcu2_acu_sweep_sta_msg);
    break;
    

    case ScuWorkTimeFb::ID:
    scu_work_time_fb_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    scu_work_time_fb_entity_.update_bytes(byte_temp);
    scu_work_time_fb_entity_.Parse();

    scu_work_time_fb_msg.header = header;
    scu_work_time_fb_msg.battery_voltage = scu_work_time_fb_entity_.battery_voltage_;
scu_work_time_fb_msg.work_time = scu_work_time_fb_entity_.work_time_;

    scu_work_time_fb_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::ScuWorkTimeFb>(scu_work_time_fb_msg);
    break;
    

    case Vcu2AcuSweepWorkSta::ID:
    vcu2_acu_sweep_work_sta_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    vcu2_acu_sweep_work_sta_entity_.update_bytes(byte_temp);
    vcu2_acu_sweep_work_sta_entity_.Parse();

    vcu2_acu_sweep_work_sta_msg.header = header;
    vcu2_acu_sweep_work_sta_msg.sweep_emergency_sig_fb = vcu2_acu_sweep_work_sta_entity_.sweep_emergency_sig_fb_;
vcu2_acu_sweep_work_sta_msg.vcu_sweep_plate_up_down_sta_fb = vcu2_acu_sweep_work_sta_entity_.vcu_sweep_plate_up_down_sta_fb_;
vcu2_acu_sweep_work_sta_msg.vcu_auto_garbage_dump_sta_fb = vcu2_acu_sweep_work_sta_entity_.vcu_auto_garbage_dump_sta_fb_;
vcu2_acu_sweep_work_sta_msg.vcu_auto_cleaning_sta_fb = vcu2_acu_sweep_work_sta_entity_.vcu_auto_cleaning_sta_fb_;

    vcu2_acu_sweep_work_sta_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::Vcu2AcuSweepWorkSta>(vcu2_acu_sweep_work_sta_msg);
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
vehicle_mileage_fb_msg.vehicle_o_d_o = vehicle_mileage_fb_entity_.vehicle_o_d_o_;

    vehicle_mileage_fb_ptr_ = std::make_shared<pix_sweeping_driver_msgs::msg::VehicleMileageFb>(vehicle_mileage_fb_msg);
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
  
    const double vcu2_acu_chassis_err_code1_report_delta_time_ms =
    (current_time - vcu2_acu_chassis_err_code1_received_time_).seconds() * 1000.0;
    if(vcu2_acu_chassis_err_code1_report_delta_time_ms>param_.report_timeout_ms || vcu2_acu_chassis_err_code1_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu2_acu_chassis_err_code1 report timeout = %f ms.", vcu2_acu_chassis_err_code1_report_delta_time_ms);
    }else{
        vcu2_acu_chassis_err_code1_pub_->publish(*vcu2_acu_chassis_err_code1_ptr_);
    }
    
    const double vcu2_acu_chassis_err_code2_report_delta_time_ms =
    (current_time - vcu2_acu_chassis_err_code2_received_time_).seconds() * 1000.0;
    if(vcu2_acu_chassis_err_code2_report_delta_time_ms>param_.report_timeout_ms || vcu2_acu_chassis_err_code2_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu2_acu_chassis_err_code2 report timeout = %f ms.", vcu2_acu_chassis_err_code2_report_delta_time_ms);
    }else{
        vcu2_acu_chassis_err_code2_pub_->publish(*vcu2_acu_chassis_err_code2_ptr_);
    }
    
    const double vcu2_acu_chassis_err_code3_report_delta_time_ms =
    (current_time - vcu2_acu_chassis_err_code3_received_time_).seconds() * 1000.0;
    if(vcu2_acu_chassis_err_code3_report_delta_time_ms>param_.report_timeout_ms || vcu2_acu_chassis_err_code3_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu2_acu_chassis_err_code3 report timeout = %f ms.", vcu2_acu_chassis_err_code3_report_delta_time_ms);
    }else{
        vcu2_acu_chassis_err_code3_pub_->publish(*vcu2_acu_chassis_err_code3_ptr_);
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
    
    const double vcu2_acu_sweep_sta_fb_report_delta_time_ms =
    (current_time - vcu2_acu_sweep_sta_fb_received_time_).seconds() * 1000.0;
    if(vcu2_acu_sweep_sta_fb_report_delta_time_ms>param_.report_timeout_ms || vcu2_acu_sweep_sta_fb_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu2_acu_sweep_sta_fb report timeout = %f ms.", vcu2_acu_sweep_sta_fb_report_delta_time_ms);
    }else{
        vcu2_acu_sweep_sta_fb_pub_->publish(*vcu2_acu_sweep_sta_fb_ptr_);
    }
    
    const double vcu2_acu_sweep_fan_sta_report_delta_time_ms =
    (current_time - vcu2_acu_sweep_fan_sta_received_time_).seconds() * 1000.0;
    if(vcu2_acu_sweep_fan_sta_report_delta_time_ms>param_.report_timeout_ms || vcu2_acu_sweep_fan_sta_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu2_acu_sweep_fan_sta report timeout = %f ms.", vcu2_acu_sweep_fan_sta_report_delta_time_ms);
    }else{
        vcu2_acu_sweep_fan_sta_pub_->publish(*vcu2_acu_sweep_fan_sta_ptr_);
    }
    
    const double vcu2_acu_sweep_sta_report_delta_time_ms =
    (current_time - vcu2_acu_sweep_sta_received_time_).seconds() * 1000.0;
    if(vcu2_acu_sweep_sta_report_delta_time_ms>param_.report_timeout_ms || vcu2_acu_sweep_sta_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu2_acu_sweep_sta report timeout = %f ms.", vcu2_acu_sweep_sta_report_delta_time_ms);
    }else{
        vcu2_acu_sweep_sta_pub_->publish(*vcu2_acu_sweep_sta_ptr_);
    }
    
    const double scu_work_time_fb_report_delta_time_ms =
    (current_time - scu_work_time_fb_received_time_).seconds() * 1000.0;
    if(scu_work_time_fb_report_delta_time_ms>param_.report_timeout_ms || scu_work_time_fb_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "scu_work_time_fb report timeout = %f ms.", scu_work_time_fb_report_delta_time_ms);
    }else{
        scu_work_time_fb_pub_->publish(*scu_work_time_fb_ptr_);
    }
    
    const double vcu2_acu_sweep_work_sta_report_delta_time_ms =
    (current_time - vcu2_acu_sweep_work_sta_received_time_).seconds() * 1000.0;
    if(vcu2_acu_sweep_work_sta_report_delta_time_ms>param_.report_timeout_ms || vcu2_acu_sweep_work_sta_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "vcu2_acu_sweep_work_sta report timeout = %f ms.", vcu2_acu_sweep_work_sta_report_delta_time_ms);
    }else{
        vcu2_acu_sweep_work_sta_pub_->publish(*vcu2_acu_sweep_work_sta_ptr_);
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
    
  
}

} // namespace report_parser
} // namespace pix_sweeping_driver
