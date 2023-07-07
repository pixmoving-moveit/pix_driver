/******************************************************************************
 fulongma
 *****************************************************************************/
#include "eps_receive2_315.hpp"


Epsreceive2315::Epsreceive2315() {}
int32_t Epsreceive2315::ID = 0x315;

void Epsreceive2315::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Epsreceive2315::Parse() {
  vech_value = Vechvalue();
  vech_enable = Vechenable();
}


// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'Vech_value', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|250]', 'physical_unit': 'km/h', 'precision': 0.00390625, 'type': 'double'}
double Epsreceive2315::Vechvalue() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.003906;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'Vech_enable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Epsreceive2315::Vechenable() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

