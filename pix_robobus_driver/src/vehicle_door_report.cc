#include <pix_robobus_driver/vehicle_door_report.hpp>


VehicleDoorReport::VehicleDoorReport() {}

void VehicleDoorReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void VehicleDoorReport::Parse() {
  door_report_sta_ = doorreportsta();
  door_open_timeout_ = dooropentimeout();
  door_open_sta_ = dooropensta();
  door_open_inplace_ = dooropeninplace();
  door_close_timeout_ = doorclosetimeout();
  door_close_sta_ = doorclosesta();
  door_close_inplace_ = doorcloseinplace();
  door_button_enable_ = doorbuttonenable();
}


// config detail: {'bit': 9, 'is_signed_var': False, 'len': 2, 'name': 'door_report_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VehicleDoorReport::doorreportsta() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'door_open_timeout', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VehicleDoorReport::dooropentimeout() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'door_open_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VehicleDoorReport::dooropensta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'door_open_inplace', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VehicleDoorReport::dooropeninplace() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name': 'door_close_timeout', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VehicleDoorReport::doorclosetimeout() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'door_close_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VehicleDoorReport::doorclosesta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'door_close_inplace', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VehicleDoorReport::doorcloseinplace() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 6, 'is_signed_var': False, 'len': 1, 'name': 'door_button_enable', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VehicleDoorReport::doorbuttonenable() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

