#include <pix_robobus_driver/brake_command.hpp>

int32_t BrakeCommand::ID = 0x101;

// public
BrakeCommand::BrakeCommand() { Reset(); }

void BrakeCommand::UpdateData(bool aeb_en_ctrl, double brake_dec, int check_sum101, double brake_pedal_target, bool brake_en_ctrl) {
  set_p_aeb_en_ctrl(aeb_en_ctrl);
  set_p_brake_dec(brake_dec);
  set_p_check_sum101(check_sum101);
  set_p_brake_pedal_target(brake_pedal_target);
  set_p_brake_en_ctrl(brake_en_ctrl);
}

void BrakeCommand::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * BrakeCommand::get_data()
{
  return data;
}



// config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'aeb_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void BrakeCommand::set_p_aeb_en_ctrl(bool aeb_en_ctrl) {
  int x = aeb_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 1, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name': 'brake_dec', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
void BrakeCommand::set_p_brake_dec(double brake_dec) {
  // brake_dec = ProtocolData::BoundedValue(0.0, 10.0, brake_dec);
  int x = brake_dec / 0.010000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(a);
  to_set0.set_value(t, 6, 2);
  data[2] += to_set0.return_byte_t();
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[1] += to_set1.return_byte_t();
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum101', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void BrakeCommand::set_p_check_sum101(int check_sum101) {
  // check_sum101 = ProtocolData::BoundedValue(0, 255, check_sum101);
  int x = check_sum101;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'brake_pedal_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void BrakeCommand::set_p_brake_pedal_target(double brake_pedal_target) {
  // brake_pedal_target = ProtocolData::BoundedValue(0.0, 100.0, brake_pedal_target);
  int x = brake_pedal_target / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[4] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[3] += to_set1.return_byte_t();
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'brake_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void BrakeCommand::set_p_brake_en_ctrl(bool brake_en_ctrl) {
  int x = brake_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}


