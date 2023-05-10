#include <pix_robobus_driver/vcu_report.hpp>


VcuReport::VcuReport() {}

void VcuReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void VcuReport::Parse() {
  allow_self_driving_run_ = allowselfdrivingrun();
  vcu_chassis_estop_sta_fb_ = vcuchassisestopstafb();
  clearance_lamp_actual_ = clearancelampactual();
  car_work_state_ = carworkstate();
  car_power_state_ = carpowerstate();
  aeb_trigger_state_ = aebtriggerstate();
  brake_light_actual_ = brakelightactual();
  headlight_actual_ = headlightactual();
  turn_light_actual_ = turnlightactual();
  vehicle_errcode_ = vehicleerrcode();
  drive_mode_status_ = drivemodestatus();
  steer_mode_status_ = steermodestatus();
  vehicle_mode_state_ = vehiclemodestate();
  vehicle_front_crash_state_ = vehiclefrontcrashstate();
  back_crash_state_ = backcrashstate();
  aeb_state_ = aebstate();
  vehicle_acc_ = vehicleacc();
  vehicle_speed_ = vehiclespeed();
}


// config detail: {'bit': 54, 'is_signed_var': False, 'len': 1, 'name': 'allow_self_driving_run', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::allowselfdrivingrun() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 3, 'name': 'vcu_chassis_estop_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::vcuchassisestopstafb() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(5, 3);

  int ret = x;
  return ret;
}

// config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name': 'clearance_lamp_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::clearancelampactual() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 51, 'is_signed_var': False, 'len': 4, 'name': 'car_work_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::carworkstate() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 53, 'is_signed_var': False, 'len': 2, 'name': 'car_power_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::carpowerstate() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name': 'aeb_trigger_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::aebtriggerstate() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::brakelightactual() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name': 'headlight_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::headlightactual() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 57, 'is_signed_var': False, 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::turnlightactual() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'vehicle_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::vehicleerrcode() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 3, 'name': 'drive_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::drivemodestatus() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(5, 3);

  int ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 3, 'name': 'steer_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::steermodestatus() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 2, 'name': 'vehicle_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::vehiclemodestate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(3, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'vehicle_front_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::vehiclefrontcrashstate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'back_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::backcrashstate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'aeb_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::aebstate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'vehicle_acc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
double VcuReport::vehicleacc() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'vehicle_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double VcuReport::vehiclespeed() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

