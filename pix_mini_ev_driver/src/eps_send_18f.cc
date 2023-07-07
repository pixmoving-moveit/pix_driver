/******************************************************************************
 fulongma
 *****************************************************************************/
#include "eps_send_18f.hpp"


Epssend18f::Epssend18f() {}
int32_t Epssend18f::ID = 0x18F;

void Epssend18f::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Epssend18f::Parse() {
  xjf = XJF();
  real_current = realcurrent();
  real_angle = realangle();
  ecu_temputer = ECUtemputer();
  ecu_state = ECUstate();
}


// config detail: {'bit': 56, 'is_signed_var': False, 'len': 2, 'name': 'XJF', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Epssend18f::XJF() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 16, 'name': 'real_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|80]', 'physical_unit': 'A', 'precision': 0.001, 'type': 'double'}
double Epssend18f::realcurrent() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'real_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Epssend18f::realangle() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'ECU_temputer', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-40|150]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Epssend18f::ECUtemputer() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'ECU_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Epssend18f::ECUstate() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

