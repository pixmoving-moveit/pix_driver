#include <pix_sweeping_driver/throttle_command.hpp>

int32_t ThrottleCommand::ID = 0x100;

// public
ThrottleCommand::ThrottleCommand() { Reset(); }

void ThrottleCommand::UpdateData(int check_sum_100, double dirve_speed_target, double dirve_throttle_pedal_target, bool dirve_en_ctrl) {
  set_p_check_sum_100(check_sum_100);
  set_p_dirve_speed_target(dirve_speed_target);
  set_p_dirve_throttle_pedal_target(dirve_throttle_pedal_target);
  set_p_dirve_en_ctrl(dirve_en_ctrl);
}

void ThrottleCommand::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * ThrottleCommand::get_data()
{
  return data;
}



// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum_100', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void ThrottleCommand::set_p_check_sum_100(int check_sum_100) {
  // check_sum_100 = ProtocolData::BoundedValue(0, 255, check_sum_100);
  int x = check_sum_100;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 12, 'name': 'dirve_speed_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|40.95]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
void ThrottleCommand::set_p_dirve_speed_target(double dirve_speed_target) {
  // dirve_speed_target = ProtocolData::BoundedValue(0.0, 40.95, dirve_speed_target);
  int x = dirve_speed_target / 0.010000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(a);
  to_set0.set_value(t, 4, 4);
  data[5] += to_set0.return_byte_t();
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[4] += to_set1.return_byte_t();
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'dirve_throttle_pedal_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void ThrottleCommand::set_p_dirve_throttle_pedal_target(double dirve_throttle_pedal_target) {
  // dirve_throttle_pedal_target = ProtocolData::BoundedValue(0.0, 100.0, dirve_throttle_pedal_target);
  int x = dirve_throttle_pedal_target / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[3] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[2] += to_set1.return_byte_t();
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'dirve_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void ThrottleCommand::set_p_dirve_en_ctrl(bool dirve_en_ctrl) {
  int x = dirve_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}


