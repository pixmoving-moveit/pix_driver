#include <pix_sweeping_driver/park_report.hpp>


ParkReport::ParkReport() {}

void ParkReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void ParkReport::Parse() {
  park_flt_ = parkflt();
  parking_actual_ = parkingactual();
}


// config detail: {'bit': 15, 'description': '驻车故障', 'is_signed_var': True, 'len': 8, 'name': 'park_flt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ParkReport::parkflt() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '实际驻车状态', 'enum': {0: 'RELEASE', 1: 'PARKINGTRIGGER'}, 'is_signed_var': True, 'len': 1, 'name': 'parking_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int ParkReport::parkingactual() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  x <<= 31;
  x >>= 31;

  int ret =  static_cast<int>(x);
  return ret;
}

