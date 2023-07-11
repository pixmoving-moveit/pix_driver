#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class VcuReport {
public:
    static const uint32_t ID = 0x505;
    VcuReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int turn_light_actual_;
    bool aeb_trigger_state_;
    bool headlight_actual_;
    int car_work_state_;
    int car_power_state_;
    int vehicle_errcode_;
    bool aeb_brake_state_;
    bool front_crash_state_;
    bool back_crash_state_;
    int vehicle_mode_state_;
    int drive_mode_status_;
    double vehicle_speed_;
    int steer_mode_status_;
    bool brake_light_actual_;
    double vehicle_acc_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 57, 'description': '转向灯实际状态', 'enum': {0: 'TURNLAMPSTSOFF', 1: 'LEFTTURNLAMPSTSON', 2: 'RIGHTTURNLAMPSTSON', 3: 'HAZARDWARNINGLAMPSTSON'}, 'is_signed_var': True, 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int turnlightactual();

  // config detail: {'bit': 58, 'description': 'AEB触发状态', 'is_signed_var': True, 'len': 1, 'name': 'aeb_trigger_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool aebtriggerstate();

  // config detail: {'bit': 59, 'description': '大灯实际状态', 'is_signed_var': True, 'len': 1, 'name': 'headlight_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool headlightactual();

  // config detail: {'bit': 53, 'description': '车辆工作状态', 'enum': {0: 'INIT', 1: 'WORK1', 2: 'WORK2', 3: 'WORK3', 4: 'WORK', 5: 'ESTOP', 6: 'ERROR', 7: 'CRASH'}, 'is_signed_var': True, 'len': 4, 'name': 'car_work_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int carworkstate();

  // config detail: {'bit': 49, 'description': '车辆上电状态', 'enum': {0: 'OFF', 1: 'ON', 2: 'READY'}, 'is_signed_var': True, 'len': 2, 'name': 'car_power_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int carpowerstate();

  // config detail: {'bit': 47, 'description': '车辆故障码', 'is_signed_var': True, 'len': 8, 'name': 'vehicle_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vehicleerrcode();

  // config detail: {'bit': 32, 'description': 'AEB制动状态', 'is_signed_var': True, 'len': 1, 'name': 'aeb_brake_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool aebbrakestate();

  // config detail: {'bit': 33, 'description': '前碰撞状态', 'is_signed_var': True, 'len': 1, 'name': 'front_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool frontcrashstate();

  // config detail: {'bit': 34, 'description': '后碰撞状态', 'is_signed_var': True, 'len': 1, 'name': 'back_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool backcrashstate();

  // config detail: {'bit': 36, 'description': '车辆模式状态', 'enum': {0: 'MANUAL_REMOTE_MODE', 1: 'AUTO_MODE', 2: 'EMERGENCY_MODE', 3: 'STANDBY_MODE'}, 'is_signed_var': True, 'len': 2, 'name': 'vehicle_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int vehiclemodestate();

  // config detail: {'bit': 39, 'description': '行驶模式状态', 'enum': {0: 'THROTTLE_PADDLE_DRIVE_MODE', 1: 'SPEED_DRIVE_MODE'}, 'is_signed_var': True, 'len': 3, 'name': 'drive_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int drivemodestatus();

  // config detail: {'bit': 23, 'description': '车辆速度', 'is_signed_var': True, 'len': 16, 'name': 'vehicle_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double vehiclespeed();

  // config detail: {'bit': 10, 'description': '转向模式', 'enum': {0: 'STANDARD_STEER_MODE', 1: 'NON_DIRECTION_STEER_MODE', 2: 'SYNC_DIRECTION_STEER_MODE'}, 'is_signed_var': False, 'len': 3, 'name': 'steer_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int steermodestatus();

  // config detail: {'bit': 11, 'description': '制动灯实际状态', 'is_signed_var': False, 'len': 1, 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool brakelightactual();

  // config detail: {'bit': 7, 'description': '车辆加速度', 'is_signed_var': True, 'len': 12, 'name': 'vehicle_acc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  double vehicleacc();
};



