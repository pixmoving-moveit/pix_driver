#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class VcuReport {
public:
    static const uint32_t ID = 0x505;
    VcuReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    bool allow_self_driving_run_;
    int vcu_chassis_estop_sta_fb_;
    bool clearance_lamp_actual_;
    int car_work_state_;
    int car_power_state_;
    bool aeb_trigger_state_;
    bool brake_light_actual_;
    bool headlight_actual_;
    int turn_light_actual_;
    int vehicle_errcode_;
    int drive_mode_status_;
    int steer_mode_status_;
    int vehicle_mode_state_;
    bool vehicle_front_crash_state_;
    bool back_crash_state_;
    bool aeb_state_;
    double vehicle_acc_;
    double vehicle_speed_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 54, 'is_signed_var': False, 'len': 1, 'name': 'allow_self_driving_run', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool allowselfdrivingrun();

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 3, 'name': 'vcu_chassis_estop_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vcuchassisestopstafb();

  // config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name': 'clearance_lamp_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool clearancelampactual();

  // config detail: {'bit': 51, 'is_signed_var': False, 'len': 4, 'name': 'car_work_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int carworkstate();

  // config detail: {'bit': 53, 'is_signed_var': False, 'len': 2, 'name': 'car_power_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int carpowerstate();

  // config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name': 'aeb_trigger_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool aebtriggerstate();

  // config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool brakelightactual();

  // config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name': 'headlight_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool headlightactual();

  // config detail: {'bit': 57, 'is_signed_var': False, 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int turnlightactual();

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'vehicle_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vehicleerrcode();

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 3, 'name': 'drive_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int drivemodestatus();

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 3, 'name': 'steer_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int steermodestatus();

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 2, 'name': 'vehicle_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vehiclemodestate();

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'vehicle_front_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool vehiclefrontcrashstate();

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'back_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool backcrashstate();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'aeb_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool aebstate();

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'vehicle_acc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  double vehicleacc();

  // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'vehicle_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double vehiclespeed();
};



