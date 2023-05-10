#include <pix_robobus_driver/gear_command.hpp>

int32_t GearCommand::ID = 0x103;

// public
GearCommand::GearCommand() { Reset(); }

void GearCommand::UpdateData(int gear_target, bool gear_en_ctrl, int check_sum103) {
  set_p_gear_target(gear_target);
  set_p_gear_en_ctrl(gear_en_ctrl);
  set_p_check_sum103(check_sum103);
}

void GearCommand::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * GearCommand::get_data()
{
  return data;
}



// config detail: {'bit': 10, 'is_signed_var': False, 'len': 3, 'name': 'gear_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|4]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void GearCommand::set_p_gear_target(int gear_target) {
  // gear_target = ProtocolData::BoundedValue(0, 4, gear_target);
  int x = gear_target;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'gear_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void GearCommand::set_p_gear_en_ctrl(bool gear_en_ctrl) {
  int x = gear_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum103', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void GearCommand::set_p_check_sum103(int check_sum103) {
  // check_sum103 = ProtocolData::BoundedValue(0, 255, check_sum103);
  int x = check_sum103;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}


