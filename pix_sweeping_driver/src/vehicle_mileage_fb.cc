#include <pix_sweeping_driver/vehicle_mileage_fb.hpp>


VehicleMileageFb::VehicleMileageFb() {}

void VehicleMileageFb::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void VehicleMileageFb::Parse() {
  vehicle_sub_mileage_ = vehiclesubmileage();
  vehicle_o_d_o_ = vehicleodo();
}


// config detail: {'bit': 39, 'is_signed_var': True, 'len': 24, 'name': 'vehicle_sub_mileage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|167772.15]', 'physical_unit': 'km', 'precision': 0.01, 'type': 'double'}
double VehicleMileageFb::vehiclesubmileage() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(*(bytes + 6));
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 8;
  x >>= 8;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': True, 'len': 24, 'name': 'vehicle_o_d_o', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1677721.5]', 'physical_unit': 'km', 'precision': 0.1, 'type': 'double'}
double VehicleMileageFb::vehicleodo() {
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

