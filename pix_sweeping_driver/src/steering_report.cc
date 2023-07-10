#include <pix_sweeping_driver/steering_report.hpp>


SteeringReport::SteeringReport() {}

void SteeringReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void SteeringReport::Parse() {
  steer_angle_speed_actual_ = steeranglespeedactual();
  steer_angle_rear_actual_ = steeranglerearactual();
  steer_angle_actual_ = steerangleactual();
  steer_flt2_ = steerflt2();
  steer_flt1_ = steerflt1();
  steer_en_state_ = steerenstate();
}


// config detail: {'bit': 63, 'is_signed_var': True, 'len': 8, 'name': 'steer_angle_speed_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steeranglespeedactual() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': True, 'len': 16, 'name': 'steer_angle_rear_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steeranglerearactual() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': True, 'len': 16, 'name': 'steer_angle_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerangleactual() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 8, 'name': 'steer_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerflt2() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': True, 'len': 8, 'name': 'steer_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerflt1() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 1, 'is_signed_var': True, 'len': 2, 'name': 'steer_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int SteeringReport::steerenstate() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  x <<= 30;
  x >>= 30;

  int ret = x;
  return ret;
}

