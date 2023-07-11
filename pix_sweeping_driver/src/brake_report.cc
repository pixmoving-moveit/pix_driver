#include <pix_sweeping_driver/brake_report.hpp>


BrakeReport::BrakeReport() {}

void BrakeReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void BrakeReport::Parse() {
  brake_pedal_actual_ = brakepedalactual();
  brake_flt2_ = brakeflt2();
  brake_flt1_ = brakeflt1();
  brake_en_state_ = brakeenstate();
}


// config detail: {'bit': 31, 'description': '制动踏板实际值', 'is_signed_var': True, 'len': 16, 'name': 'brake_pedal_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double BrakeReport::brakepedalactual() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 23, 'description': '制动故障码2', 'is_signed_var': False, 'len': 8, 'name': 'brake_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int BrakeReport::brakeflt2() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'description': '制动故障码1', 'is_signed_var': True, 'len': 8, 'name': 'brake_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int BrakeReport::brakeflt1() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 1, 'description': '制动使能状态', 'enum': {0: 'MANUAL', 1: 'AUTO', 2: 'TAKEOVER', 3: 'STANDBY'}, 'is_signed_var': True, 'len': 2, 'name': 'brake_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int BrakeReport::brakeenstate() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  x <<= 30;
  x >>= 30;

  int ret =  static_cast<int>(x);
  return ret;
}

