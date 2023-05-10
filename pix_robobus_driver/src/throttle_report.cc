#include <pix_robobus_driver/throttle_report.hpp>


ThrottleReport::ThrottleReport() {}

void ThrottleReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void ThrottleReport::Parse() {
  dirve_throttle_pedal_actual_ = dirvethrottlepedalactual();
  dirve_flt2_ = dirveflt2();
  dirve_flt1_ = dirveflt1();
  dirve_en_state_ = dirveenstate();
}


// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'dirve_throttle_pedal_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double ThrottleReport::dirvethrottlepedalactual() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'dirve_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ThrottleReport::dirveflt2() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'dirve_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ThrottleReport::dirveflt1() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 2, 'name': 'dirve_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ThrottleReport::dirveenstate() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

