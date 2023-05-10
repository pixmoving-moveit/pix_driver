#include <pix_robobus_driver/wheel_speed_report.hpp>


WheelSpeedReport::WheelSpeedReport() {}

void WheelSpeedReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void WheelSpeedReport::Parse() {
  wheel_speed_rr_ = wheelspeedrr();
  wheel_speed_rl_ = wheelspeedrl();
  wheel_speed_fr_ = wheelspeedfr();
  wheel_speed_fl_ = wheelspeedfl();
}


// config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_rr', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double WheelSpeedReport::wheelspeedrr() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 7));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_rl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double WheelSpeedReport::wheelspeedrl() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_fr', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double WheelSpeedReport::wheelspeedfr() {
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

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_fl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double WheelSpeedReport::wheelspeedfl() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

