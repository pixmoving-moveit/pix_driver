#include "steeringreport_314.hpp"


Steeringreport314::Steeringreport314() {}
int32_t Steeringreport314::ID = 0x314;

void Steeringreport314::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Steeringreport314::Parse() {
  work_state = workstate();
  tar_angle = tarangle();
  cail_sas = cailsas();
}


// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'work_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Steeringreport314::workstate() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'tar_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Steeringreport314::tarangle() {
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

// config detail: {'bit': 1, 'enum': {0: 'CAIL_SAS_DEFAULT', 1: 'CAIL_SAS_CAIL'}, 'is_signed_var': False, 'len': 2, 'name': 'cail_sas', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Steeringreport314::cailsas() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

