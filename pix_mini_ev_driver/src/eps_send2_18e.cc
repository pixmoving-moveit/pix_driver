/******************************************************************************
 fulongma
 *****************************************************************************/
#include "eps_send2_18e.hpp"


Epssend218e::Epssend218e() {}
int32_t Epssend218e::ID = 0x18E;

void Epssend218e::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Epssend218e::Parse() {
  work_mode = workmode();
  torger_value = Torgervalue();
  tar_current = Tarcurrent();
  steer_speed = Steerspeed();
}


// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'work_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Epssend218e::workmode() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': True, 'len': 16, 'name': 'Torger_value', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-6|6]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
double Epssend218e::Torgervalue() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': True, 'len': 16, 'name': 'Tar_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-40|40]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Epssend218e::Tarcurrent() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': True, 'len': 16, 'name': 'Steer_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-4096|4096]', 'physical_unit': '\u3000deg/s', 'precision': 0.1, 'type': 'double'}
double Epssend218e::Steerspeed() {
  Byte t0(*(bytes + 8));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 7));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

