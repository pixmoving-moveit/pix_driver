#include <pix_sweeping_driver/vcu_report.hpp>


VcuReport::VcuReport() {}

void VcuReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void VcuReport::Parse() {
  turn_light_actual_ = turnlightactual();
  aeb_trigger_state_ = aebtriggerstate();
  headlight_actual_ = headlightactual();
  car_work_state_ = carworkstate();
  car_power_state_ = carpowerstate();
  vehicle_errcode_ = vehicleerrcode();
  aeb_brake_state_ = aebbrakestate();
  front_crash_state_ = frontcrashstate();
  back_crash_state_ = backcrashstate();
  vehicle_mode_state_ = vehiclemodestate();
  drive_mode_status_ = drivemodestatus();
  vehicle_speed_ = vehiclespeed();
  steer_mode_status_ = steermodestatus();
  brake_light_actual_ = brakelightactual();
  vehicle_acc_ = vehicleacc();
}


// config detail: {'bit': 57, 'is_signed_var': True, 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::turnlightactual() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 2);

  x <<= 30;
  x >>= 30;

  int ret = x;
  return ret;
}

// config detail: {'bit': 58, 'is_signed_var': True, 'len': 1, 'name': 'aeb_trigger_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::aebtriggerstate() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(2, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'is_signed_var': True, 'len': 1, 'name': 'headlight_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::headlightactual() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(3, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 53, 'is_signed_var': True, 'len': 4, 'name': 'car_work_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::carworkstate() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(2, 4);

  x <<= 28;
  x >>= 28;

  int ret = x;
  return ret;
}

// config detail: {'bit': 49, 'is_signed_var': True, 'len': 2, 'name': 'car_power_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::carpowerstate() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 2);

  x <<= 30;
  x >>= 30;

  int ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': True, 'len': 8, 'name': 'vehicle_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::vehicleerrcode() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': True, 'len': 1, 'name': 'aeb_brake_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::aebbrakestate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 33, 'is_signed_var': True, 'len': 1, 'name': 'front_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::frontcrashstate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(1, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 34, 'is_signed_var': True, 'len': 1, 'name': 'back_crash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::backcrashstate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(2, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': True, 'len': 2, 'name': 'vehicle_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::vehiclemodestate() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(3, 2);

  x <<= 30;
  x >>= 30;

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 3, 'name': 'drive_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::drivemodestatus() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(5, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
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

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 3, 'name': 'steer_mode_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VcuReport::steermodestatus() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

// config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VcuReport::brakelightactual() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(3, 1);

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

