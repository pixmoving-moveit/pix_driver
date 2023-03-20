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

#include <pix_hooke_driver/report_parser.hpp>

namespace pix_hooke_driver
{
namespace report_parser
{
ReportParser::ReportParser(): Node("report_parser")
{
  // ros params
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");
  param_.report_timeout_ms = declare_parameter("report_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);

  // initialize msg received time
  brake_sta_fb_received_time_ = this->now();
  chassis_wheel_angle_fb_received_time_ = this->now();
  chassis_wheel_rpm_fb_received_time_ = this->now();
  chassis_wheel_tire_press_fb_received_time_ = this->now();
  drive_sta_fb_received_time_ = this->now();
  power_sta_fb_received_time_ = this->now();
  steer_sta_fb_received_time_ = this->now();
  vehicle_flt_sta_received_time_ = this->now();
  vehicle_sta_fb_received_time_ = this->now();
  vehicle_work_sta_fb_received_time_ = this->now();

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
    brake_sta_fb_pub_ =
      create_publisher<V2aBrakeStaFb>("/pix_hooke/v2a_brakestafb", rclcpp::QoS{1});
    chassis_wheel_angle_fb_pub_ = create_publisher<V2aChassisWheelAngleFb>(
      "/pix_hooke/v2a_chassiswheelanglefb", rclcpp::QoS{1});
    chassis_wheel_rpm_fb_pub_ =
      create_publisher<V2aChassisWheelRpmFb>("/pix_hooke/v2a_chassiswheelrpmfb", rclcpp::QoS{1});
    chassis_wheel_tire_press_fb_pub_ = create_publisher<V2aChassisWheelTirePressFb>(
      "/pix_hooke/v2a_chassiswheeltirepressfb", rclcpp::QoS{1});
    drive_sta_fb_pub_ =
      create_publisher<V2aDriveStaFb>("/pix_hooke/v2a_drivestafb", rclcpp::QoS{1});
    power_sta_fb_pub_ =
      create_publisher<V2aPowerStaFb>("/pix_hooke/v2a_powerstafb", rclcpp::QoS{1});
    steer_sta_fb_pub_ =
      create_publisher<V2aSteerStaFb>("/pix_hooke/v2a_steerstafb", rclcpp::QoS{1});
    vehicle_flt_sta_pub_ =
      create_publisher<V2aVehicleFltSta>("/pix_hooke/v2a_vehiclefltsta", rclcpp::QoS{1});
    vehicle_sta_fb_pub_ =
      create_publisher<V2aVehicleStaFb>("/pix_hooke/v2a_vehiclestafb", rclcpp::QoS{1});
    vehicle_work_sta_fb_pub_ =
      create_publisher<V2aVehicleWorkStaFb>("/pix_hooke/v2a_vehicleworkstafb", rclcpp::QoS{1});
  }
  {
    // timer
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
      std::bind(&ReportParser::timerCallback, this));
  }
}

void ReportParser::callbackCan(const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  std_msgs::msg::Header header;
  header.frame_id = param_.base_frame_id;
  header.stamp = msg->header.stamp;

  V2aBrakeStaFb brake_sta_fb_msg;
  V2aChassisWheelAngleFb chassis_wheel_angle_fb_msg;
  V2aChassisWheelRpmFb chassis_wheel_rpm_fb_msg;
  V2aChassisWheelTirePressFb chassis_wheel_tire_press_fb_msg;
  V2aDriveStaFb drive_sta_fb_msg;
  V2aPowerStaFb power_sta_fb_msg;
  V2aSteerStaFb steer_sta_fb_msg;
  V2aVehicleFltSta vehicle_flt_sta_msg;
  V2aVehicleStaFb vehicle_sta_fb_msg;
  V2aVehicleWorkStaFb vehicle_work_sta_fb_msg;

  uint8_t byte_temp[8];
  switch (msg->id)
  {
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
  case V2abrakestafb531::ID:
    brake_sta_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_brakestafb_531_entity_.update_bytes(byte_temp);
    v2a_brakestafb_531_entity_.Parse();

    brake_sta_fb_msg.header = header;
    brake_sta_fb_msg.vcu_chassis_brake_en_sta = v2a_brakestafb_531_entity_.vcu_chassisbrakeensta;
		brake_sta_fb_msg.vcu_vehicle_brake_lamp_fb = v2a_brakestafb_531_entity_.vcu_vehiclebrakelampfb;
		brake_sta_fb_msg.vcu_chassis_epb_fb = v2a_brakestafb_531_entity_.vcu_chassisepbfb;
		brake_sta_fb_msg.vcu_chassis_brake_padl_fb = v2a_brakestafb_531_entity_.vcu_chassisbrakepadlfb;
		brake_sta_fb_msg.vcu_aeb_en_sta_fb = v2a_brakestafb_531_entity_.vcu_aebenstafb;
		brake_sta_fb_msg.vcu_aeb_trigger_sta_fb = v2a_brakestafb_531_entity_.vcu_aebtriggerstafb;
    brake_sta_fb_ptr_ = std::make_shared<V2aBrakeStaFb>(brake_sta_fb_msg);
    break; 
  case V2asteerstafb532::ID:
    steer_sta_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_steerstafb_532_entity_.update_bytes(byte_temp);
    v2a_steerstafb_532_entity_.Parse();
    steer_sta_fb_msg.header = header;
		steer_sta_fb_msg.vcu_chassis_steer_en_sta = v2a_steerstafb_532_entity_.vcu_chassissteerensta;
		steer_sta_fb_msg.vcu_chassis_steer_slopover = v2a_steerstafb_532_entity_.vcu_chassissteerslopover;
		steer_sta_fb_msg.vcu_chassis_steer_work_mode = v2a_steerstafb_532_entity_.vcu_chassissteerworkmode;
		steer_sta_fb_msg.vcu_chassis_steer_mode_fb = v2a_steerstafb_532_entity_.vcu_chassissteermodefb;
		steer_sta_fb_msg.vcu_chassis_steer_angle_fb = v2a_steerstafb_532_entity_.vcu_chassissteeranglefb;
		steer_sta_fb_msg.vcu_chassis_steer_angle_rear_fb = v2a_steerstafb_532_entity_.vcu_chassissteeranglerearfb;
		steer_sta_fb_msg.vcu_chassis_steer_angle_speed_fb = v2a_steerstafb_532_entity_.vcu_chassissteeranglespeedfb;
    steer_sta_fb_ptr_ = std::make_shared<V2aSteerStaFb>(steer_sta_fb_msg);
    break;
  case V2avehicleworkstafb534::ID:
    vehicle_work_sta_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_vehicleworkstafb_534_entity_.update_bytes(byte_temp);
    v2a_vehicleworkstafb_534_entity_.Parse();
    vehicle_work_sta_fb_msg.header = header;
		vehicle_work_sta_fb_msg.vcu_driving_mode_fb = v2a_vehicleworkstafb_534_entity_.vcu_drivingmodefb;
		vehicle_work_sta_fb_msg.vcu_chassis_power_sta_fb = v2a_vehicleworkstafb_534_entity_.vcu_chassispowerstafb;
		vehicle_work_sta_fb_msg.vcu_chassis_power_dc_sta = v2a_vehicleworkstafb_534_entity_.vcu_chassispowerdcsta;
		vehicle_work_sta_fb_msg.vcu_chassis_speed_limited_mode_fb = v2a_vehicleworkstafb_534_entity_.vcu_chassisspeedlimitedmodefb;
		vehicle_work_sta_fb_msg.vcu_chassis_power_limite_sta = v2a_vehicleworkstafb_534_entity_.vcu_chassispowerlimitesta;
		vehicle_work_sta_fb_msg.vcu_sys_eco_mode = v2a_vehicleworkstafb_534_entity_.vcu_sysecomode;
		vehicle_work_sta_fb_msg.vcu_chassis_speed_limited_val_fb = v2a_vehicleworkstafb_534_entity_.vcu_chassisspeedlimitedvalfb;
		vehicle_work_sta_fb_msg.vcu_chassis_low_power_volt_sta = v2a_vehicleworkstafb_534_entity_.vcu_chassislowpowervoltsta;
		vehicle_work_sta_fb_msg.vcu_chassis_e_stop_sta_fb = v2a_vehicleworkstafb_534_entity_.vcu_chassisestopstafb;
		vehicle_work_sta_fb_msg.vcu_crash_front_sta = v2a_vehicleworkstafb_534_entity_.vcu_crashfrontsta;
		vehicle_work_sta_fb_msg.vcu_crash_rear_sta = v2a_vehicleworkstafb_534_entity_.vcu_crashrearsta;
		vehicle_work_sta_fb_msg.vcu_crash_left_sta = v2a_vehicleworkstafb_534_entity_.vcu_crashleftsta;
		vehicle_work_sta_fb_msg.vcu_crash_right_sta = v2a_vehicleworkstafb_534_entity_.vcu_crashrightsta;
		vehicle_work_sta_fb_msg.vcu_life = v2a_vehicleworkstafb_534_entity_.vcu_life;
		vehicle_work_sta_fb_msg.vcu_check_sum = v2a_vehicleworkstafb_534_entity_.vcu_checksum;
    vehicle_work_sta_fb_ptr_ = std::make_shared<V2aVehicleWorkStaFb>(vehicle_work_sta_fb_msg);
    break;
  case V2apowerstafb535::ID:
    power_sta_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_powerstafb_535_entity_.update_bytes(byte_temp);
    v2a_powerstafb_535_entity_.Parse();
    power_sta_fb_msg.header = header;
    power_sta_fb_msg.vcu_chassis_bms_reserved_1 = v2a_powerstafb_535_entity_.vcu_chassisbmsreserved_1;
		power_sta_fb_msg.vcu_chassis_power_charge_sta = v2a_powerstafb_535_entity_.vcu_chassispowerchargesta;
		power_sta_fb_msg.vcu_chassis_power_charge_sock_sta = v2a_powerstafb_535_entity_.vcu_chassispowerchargesocksta;
		power_sta_fb_msg.vcu_chassis_power_soc_fb = v2a_powerstafb_535_entity_.vcu_chassispowersocfb;
		power_sta_fb_msg.vcu_chassis_power_volt_fb = v2a_powerstafb_535_entity_.vcu_chassispowervoltfb;
		power_sta_fb_msg.vcu_chassis_power_curr_fb = v2a_powerstafb_535_entity_.vcu_chassispowercurrfb;
		power_sta_fb_msg.vcu_chassis_bms_max_temp = v2a_powerstafb_535_entity_.vcu_chassisbmsmaxtemp;
		power_sta_fb_msg.vcu_chassis_bms_reserved_2 = v2a_powerstafb_535_entity_.vcu_chassisbmsreserved_2;
    power_sta_fb_ptr_ = std::make_shared<V2aPowerStaFb>(power_sta_fb_msg);
    break;
  case V2avehiclestafb536::ID:
    vehicle_sta_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_vehiclestafb_536_entity_.update_bytes(byte_temp);
    v2a_vehiclestafb_536_entity_.Parse();
    vehicle_sta_fb_msg.header = header;
		vehicle_sta_fb_msg.vcu_vehicle_pos_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehicleposlampfb;
		vehicle_sta_fb_msg.vcu_vehicle_head_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehicleheadlampfb;
		vehicle_sta_fb_msg.vcu_vehicle_left_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehicleleftlampfb;
		vehicle_sta_fb_msg.vcu_vehicle_right_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehiclerightlampfb;
		vehicle_sta_fb_msg.vcu_vehicle_high_beam_fb = v2a_vehiclestafb_536_entity_.vcu_vehiclehighbeamfb;
		vehicle_sta_fb_msg.vcu_vehicle_fog_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehiclefoglampfb;
		vehicle_sta_fb_msg.vcu_vehicle_hazard_war_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehiclehazardwarlampfb;
		vehicle_sta_fb_msg.vcu_vehicle_body_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehiclebodylampfb;
		vehicle_sta_fb_msg.vcu_vehicle_read_lamp_fb = v2a_vehiclestafb_536_entity_.vcu_vehiclereadlampfb;
		vehicle_sta_fb_msg.acu_vehicle_window_fb = v2a_vehiclestafb_536_entity_.acu_vehiclewindowfb;
		vehicle_sta_fb_msg.vcu_vehicle_door_sta_fb = v2a_vehiclestafb_536_entity_.vcu_vehicledoorstafb;
		vehicle_sta_fb_msg.vcu_vehicle_wipers_sta_fb = v2a_vehiclestafb_536_entity_.vcu_vehiclewipersstafb;
		vehicle_sta_fb_msg.vcu_vehicle_safety_belt_1 = v2a_vehiclestafb_536_entity_.vcu_vehiclesafetybelt1;
		vehicle_sta_fb_msg.vcu_vehicle_safety_belt_2 = v2a_vehiclestafb_536_entity_.vcu_vehiclesafetybelt2;
		vehicle_sta_fb_msg.vcu_vehicle_safety_belt_3 = v2a_vehiclestafb_536_entity_.vcu_vehiclesafetybelt3;
		vehicle_sta_fb_msg.vcu_vehicle_safety_belt_4 = v2a_vehiclestafb_536_entity_.vcu_vehiclesafetybelt4;
    vehicle_sta_fb_ptr_ = std::make_shared<V2aVehicleStaFb>(vehicle_sta_fb_msg);
    break;
  case V2avehiclefltsta537::ID:
    vehicle_flt_sta_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_vehiclefltsta_537_entity_.update_bytes(byte_temp);
    v2a_vehiclefltsta_537_entity_.Parse();
    vehicle_flt_sta_msg.header = header;
    vehicle_flt_sta_msg.vcu_sys_motor_over_temp_sta = v2a_vehiclefltsta_537_entity_.vcu_sysmotorovertempsta;
		vehicle_flt_sta_msg.vcu_sys_bms_over_temp_sta = v2a_vehiclefltsta_537_entity_.vcu_sysbmsovertempsta;
		vehicle_flt_sta_msg.vcu_sys_brake_over_temp_sta = v2a_vehiclefltsta_537_entity_.vcu_sysbrakeovertempsta;
		vehicle_flt_sta_msg.vcu_sys_steer_over_temp_sta = v2a_vehiclefltsta_537_entity_.vcu_syssteerovertempsta;
		vehicle_flt_sta_msg.vcu_sys_under_volt = v2a_vehiclefltsta_537_entity_.vcu_sysundervolt;
		vehicle_flt_sta_msg.vcu_sys_flt = v2a_vehiclefltsta_537_entity_.vcu_sysflt;
		vehicle_flt_sta_msg.vcu_sys_brake_flt = v2a_vehiclefltsta_537_entity_.vcu_sysbrakeflt;
		vehicle_flt_sta_msg.vcu_sys_parking_flt = v2a_vehiclefltsta_537_entity_.vcu_sysparkingflt;
		vehicle_flt_sta_msg.vcu_sys_steer_front_flt = v2a_vehiclefltsta_537_entity_.vcu_syssteerfrontflt;
		vehicle_flt_sta_msg.vcu_sys_steer_back_flt = v2a_vehiclefltsta_537_entity_.vcu_syssteerbackflt;
		vehicle_flt_sta_msg.vcu_sys_motor_lf_flt = v2a_vehiclefltsta_537_entity_.vcu_sysmotorlfflt;
		vehicle_flt_sta_msg.vcu_sys_motor_rf_flt = v2a_vehiclefltsta_537_entity_.vcu_sysmotorrfflt;
		vehicle_flt_sta_msg.vcu_sys_motor_lr_flt = v2a_vehiclefltsta_537_entity_.vcu_sysmotorlrflt;
		vehicle_flt_sta_msg.vcu_sys_motor_rr_flt = v2a_vehiclefltsta_537_entity_.vcu_sysmotorrrflt;
		vehicle_flt_sta_msg.vcu_sys_bms_flt = v2a_vehiclefltsta_537_entity_.vcu_sysbmsflt;
		vehicle_flt_sta_msg.vcu_sys_dc_flt = v2a_vehiclefltsta_537_entity_.vcu_sysdcflt;
    vehicle_flt_sta_ptr_ = std::make_shared<V2aVehicleFltSta>(vehicle_flt_sta_msg);
    break;
  case V2achassiswheelrpmfb539::ID:
    chassis_wheel_rpm_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_chassiswheelrpmfb_539_entity_.update_bytes(byte_temp);
    v2a_chassiswheelrpmfb_539_entity_.Parse();
    chassis_wheel_rpm_fb_msg.header = header;
    chassis_wheel_rpm_fb_msg.vcu_chassis_wheel_rpm_lf = v2a_chassiswheelrpmfb_539_entity_.vcu_chassiswheelrpmlf;
		chassis_wheel_rpm_fb_msg.vcu_chassis_wheel_rpm_rf = v2a_chassiswheelrpmfb_539_entity_.vcu_chassiswheelrpmrf;
		chassis_wheel_rpm_fb_msg.vcu_chassis_wheel_rpm_lr = v2a_chassiswheelrpmfb_539_entity_.vcu_chassiswheelrpmlr;
		chassis_wheel_rpm_fb_msg.vcu_chassis_wheel_rpm_rr = v2a_chassiswheelrpmfb_539_entity_.vcu_chassiswheelrpmrr;
		
    chassis_wheel_rpm_fb_ptr_ = std::make_shared<V2aChassisWheelRpmFb>(chassis_wheel_rpm_fb_msg);
    break;
  case V2achassiswheeltirepressfb540::ID:
    chassis_wheel_tire_press_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_chassiswheeltirepressfb_540_entity_.update_bytes(byte_temp);
    v2a_chassiswheeltirepressfb_540_entity_.Parse();
    chassis_wheel_tire_press_fb_msg.header = header;
		chassis_wheel_tire_press_fb_msg.vcu_chassis_wheel_tire_press_lf = v2a_chassiswheeltirepressfb_540_entity_.vcu_chassiswheeltirepresslf;
		chassis_wheel_tire_press_fb_msg.vcu_chassis_wheel_tire_press_rf = v2a_chassiswheeltirepressfb_540_entity_.vcu_chassiswheeltirepressrf;
		chassis_wheel_tire_press_fb_msg.vcu_chassis_wheel_tire_press_lr = v2a_chassiswheeltirepressfb_540_entity_.vcu_chassiswheeltirepresslr;
		chassis_wheel_tire_press_fb_msg.vcu_chassis_wheel_tire_press_rr = v2a_chassiswheeltirepressfb_540_entity_.vcu_chassiswheeltirepressrr;
    chassis_wheel_tire_press_fb_ptr_ = std::make_shared<V2aChassisWheelTirePressFb>(chassis_wheel_tire_press_fb_msg);
    break;
  case V2achassiswheelanglefb541::ID:
    chassis_wheel_angle_fb_received_time_ = this->now();
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_chassiswheelanglefb_541_entity_.update_bytes(byte_temp);
    v2a_chassiswheelanglefb_541_entity_.Parse();
    chassis_wheel_angle_fb_msg.header = header;
		chassis_wheel_angle_fb_msg.vcu_chassis_wheel_angle_lf = v2a_chassiswheelanglefb_541_entity_.vcu_chassiswheelanglelf;
		chassis_wheel_angle_fb_msg.vcu_chassis_wheel_angle_rf = v2a_chassiswheelanglefb_541_entity_.vcu_chassiswheelanglerf;
		chassis_wheel_angle_fb_msg.vcu_chassis_wheel_angle_lr = v2a_chassiswheelanglefb_541_entity_.vcu_chassiswheelanglelr;
		chassis_wheel_angle_fb_msg.vcu_chassis_wheel_angle_rr = v2a_chassiswheelanglefb_541_entity_.vcu_chassiswheelanglerr; 
    chassis_wheel_angle_fb_ptr_ = std::make_shared<V2aChassisWheelAngleFb>(chassis_wheel_angle_fb_msg);
    break;
  default:
    break;
  }
}

void ReportParser::callbackIsPublish(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_publish_ = msg->data;
}

void ReportParser::timerCallback()
{
  if (!is_publish_) return;

  const rclcpp::Time current_time = this->now();
  
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
  // brake sta fb report
  const double brake_sta_fb_report_delta_time_ms =
    (current_time - brake_sta_fb_received_time_).seconds() * 1000.0;
  if(brake_sta_fb_report_delta_time_ms>param_.report_timeout_ms || brake_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "brake stat fb report timeout = %f ms.", brake_sta_fb_report_delta_time_ms);
  }else{
    brake_sta_fb_pub_->publish(*brake_sta_fb_ptr_);
  }
  // steer sta fb report
  const double steer_sta_fb_report_delta_time_ms =
    (current_time - steer_sta_fb_received_time_).seconds() * 1000.0;
  if(steer_sta_fb_report_delta_time_ms>param_.report_timeout_ms || steer_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "steer stat fb report timeout = %f ms.", steer_sta_fb_report_delta_time_ms);
  }else{
    steer_sta_fb_pub_->publish(*steer_sta_fb_ptr_);
  }
  // vehicle_work sta fb report
  const double vehicle_work_sta_fb_report_delta_time_ms =
    (current_time - vehicle_work_sta_fb_received_time_).seconds() * 1000.0;
  if(vehicle_work_sta_fb_report_delta_time_ms>param_.report_timeout_ms || vehicle_work_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "vehicle work stat fb report timeout = %f ms.", vehicle_work_sta_fb_report_delta_time_ms);
  }else{
    vehicle_work_sta_fb_pub_->publish(*vehicle_work_sta_fb_ptr_);
  }
  // power sta fb report
  const double power_sta_fb_report_delta_time_ms =
    (current_time - power_sta_fb_received_time_).seconds() * 1000.0;
  if(power_sta_fb_report_delta_time_ms>param_.report_timeout_ms || power_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "power stat fb report timeout = %f ms.", power_sta_fb_report_delta_time_ms);
  }else{
    power_sta_fb_pub_->publish(*power_sta_fb_ptr_);
  }
  // vehicle sta fb report
  const double vehicle_sta_fb_report_delta_time_ms =
    (current_time - vehicle_sta_fb_received_time_).seconds() * 1000.0;
  if(vehicle_sta_fb_report_delta_time_ms>param_.report_timeout_ms || vehicle_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "vehicle stat fb report timeout = %f ms.", vehicle_sta_fb_report_delta_time_ms);
  }else{
    vehicle_sta_fb_pub_->publish(*vehicle_sta_fb_ptr_);
  }
  // vehicle flt sta report
  const double vehicle_flt_sta_report_delta_time_ms =
    (current_time - vehicle_flt_sta_received_time_).seconds() * 1000.0;
  if(vehicle_flt_sta_report_delta_time_ms>param_.report_timeout_ms || vehicle_flt_sta_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "vehicle flt stat report timeout = %f ms.", vehicle_flt_sta_report_delta_time_ms);
  }else{
    vehicle_flt_sta_pub_->publish(*vehicle_flt_sta_ptr_);
  }

  // chassis wheel rpm fb report
  const double chassis_wheel_rpm_fb_report_delta_time_ms =
    (current_time - chassis_wheel_rpm_fb_received_time_).seconds() * 1000.0;
  if(chassis_wheel_rpm_fb_report_delta_time_ms>param_.report_timeout_ms || chassis_wheel_rpm_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "chassis wheel rpm fb report timeout = %f ms.", chassis_wheel_rpm_fb_report_delta_time_ms);
  }else{
    chassis_wheel_rpm_fb_pub_->publish(*chassis_wheel_rpm_fb_ptr_);
  }

  // chassis wheel tire press fb report
  const double chassis_wheel_tire_press_fb_report_delta_time_ms =
    (current_time - chassis_wheel_tire_press_fb_received_time_).seconds() * 1000.0;
  if(chassis_wheel_tire_press_fb_report_delta_time_ms>param_.report_timeout_ms || chassis_wheel_tire_press_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "chassis wheel tire press fb report timeout = %f ms.", chassis_wheel_tire_press_fb_report_delta_time_ms);
  }else{
    chassis_wheel_tire_press_fb_pub_->publish(*chassis_wheel_tire_press_fb_ptr_);
  }

  // chassis wheel angle fb report
  const double chassis_wheel_angle_fb_report_delta_time_ms =
    (current_time - chassis_wheel_angle_fb_received_time_).seconds() * 1000.0;
  if(chassis_wheel_angle_fb_report_delta_time_ms>param_.report_timeout_ms || chassis_wheel_angle_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "chassis wheel angle fb report timeout = %f ms.", chassis_wheel_angle_fb_report_delta_time_ms);
  }else{
    chassis_wheel_angle_fb_pub_->publish(*chassis_wheel_angle_fb_ptr_);
  }
}
} // namespace report_parser
} // namespace pix_hooke_driver
