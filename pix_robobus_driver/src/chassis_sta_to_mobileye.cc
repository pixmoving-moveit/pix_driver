#include <pix_robobus_driver/chassis_sta_to_mobileye.hpp>


ChassisStaToMobileye::ChassisStaToMobileye() {}

void ChassisStaToMobileye::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void ChassisStaToMobileye::Parse() {
  vehicle_speed_to_mbe_ = vehiclespeedtombe();
  brake_light_actual_ = brakelightactual();
  turn_light_actual_ = turnlightactual();
}


// config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'vehicle_speed_to_mbe', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'kph', 'precision': 0.1, 'type': 'double'}
double ChassisStaToMobileye::vehiclespeedtombe() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 18, 'is_signed_var': False, 'len': 1, 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool ChassisStaToMobileye::brakelightactual() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 17, 'is_signed_var': False, 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ChassisStaToMobileye::turnlightactual() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

