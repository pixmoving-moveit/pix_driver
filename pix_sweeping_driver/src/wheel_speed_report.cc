#include <pix_sweeping_driver/wheel_speed_report.hpp>


WheelSpeedReport::WheelSpeedReport() {}

void WheelSpeedReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void WheelSpeedReport::Parse() {
  wheel_speed_r_r_ = wheelspeedrr();
  wheel_speed_r_l_ = wheelspeedrl();
  wheel_speed_f_r_ = wheelspeedfr();
  wheel_speed_f_l_ = wheelspeedfl();
}


// config detail: {'bit': 55, 'description': '右后轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_r_r', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
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

// config detail: {'bit': 39, 'description': '左后轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_r_l', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
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

// config detail: {'bit': 23, 'description': '右前轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_f_r', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
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

// config detail: {'bit': 7, 'description': '左前轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_f_l', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
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

