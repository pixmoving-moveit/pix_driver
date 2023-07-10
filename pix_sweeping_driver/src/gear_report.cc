#include <pix_sweeping_driver/gear_report.hpp>


GearReport::GearReport() {}

void GearReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void GearReport::Parse() {
  gear_flt_ = gearflt();
  gear_actual_ = gearactual();
}


// config detail: {'bit': 15, 'is_signed_var': True, 'len': 8, 'name': 'gear_flt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int GearReport::gearflt() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 2, 'is_signed_var': True, 'len': 3, 'name': 'gear_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int GearReport::gearactual() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}
