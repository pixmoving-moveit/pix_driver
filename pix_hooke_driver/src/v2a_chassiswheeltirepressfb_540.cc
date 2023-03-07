#include <pix_hooke_driver/v2a_chassiswheeltirepressfb_540.hpp>


V2achassiswheeltirepressfb540::V2achassiswheeltirepressfb540() {}

void V2achassiswheeltirepressfb540::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2achassiswheeltirepressfb540::Parse() {
  vcu_chassiswheeltirepresslf = VCUChassisWheelTirePressLf();
  vcu_chassiswheeltirepressrf = VCUChassisWheelTirePressRf();
  vcu_chassiswheeltirepresslr = VCUChassisWheelTirePressLr();
  vcu_chassiswheeltirepressrr = VCUChassisWheelTirePressRr();
}


// config detail: {'bit': 0, 'description': '左前轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressLf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
double V2achassiswheeltirepressfb540::VCUChassisWheelTirePressLf() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 16, 'description': '右前轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressRf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
double V2achassiswheeltirepressfb540::VCUChassisWheelTirePressRf() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 32, 'description': '左后轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressLr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
double V2achassiswheeltirepressfb540::VCUChassisWheelTirePressLr() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 48, 'description': '右后轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressRr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
double V2achassiswheeltirepressfb540::VCUChassisWheelTirePressRr() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

