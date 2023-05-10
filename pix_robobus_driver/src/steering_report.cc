#include <pix_robobus_driver/steering_report.hpp>


SteeringReport::SteeringReport() {}

void SteeringReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void SteeringReport::Parse() {
  steer_angle_speed_set_val_ = steeranglespeedsetval();
  steer_flt2_ = steerflt2();
  steer_flt1_ = steerflt1();
  steer_en_state_ = steerenstate();
  steer_angle_actual_ = steerangleactual();
}


// config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'steer_angle_speed_set_val', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|500]', 'physical_unit': 'deg/s', 'precision': 2.0, 'type': 'double'}
double SteeringReport::steeranglespeedsetval() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'steer_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerflt2() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'steer_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerflt1() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 2, 'name': 'steer_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerenstate() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'steer_angle_actual', 'offset': -500.0, 'order': 'motorola', 'physical_range': '[-450|450]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerangleactual() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -500.000000;
  return ret;
}

