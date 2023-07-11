#include <pix_sweeping_driver/scu_work_time_fb.hpp>


ScuWorkTimeFb::ScuWorkTimeFb() {}

void ScuWorkTimeFb::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void ScuWorkTimeFb::Parse() {
  battery_voltage_ = batteryvoltage();
  work_time_ = worktime();
}


// config detail: {'bit': 7, 'description': '12V 系统电压', 'is_signed_var': True, 'len': 8, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|25.4]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double ScuWorkTimeFb::batteryvoltage() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 15, 'description': '工作时长', 'is_signed_var': True, 'len': 24, 'name': 'work_time', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|838860.7]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double ScuWorkTimeFb::worktime() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(*(bytes + 3));
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 8;
  x >>= 8;

  double ret = x * 0.100000;
  return ret;
}

