#include <pix_robobus_driver/park_command.hpp>

int32_t ParkCommand::ID = 0x104;

// public
ParkCommand::ParkCommand() { Reset(); }

void ParkCommand::UpdateData(int check_sum104, bool park_target, bool park_en_ctrl) {
  set_p_check_sum104(check_sum104);
  set_p_park_target(park_target);
  set_p_park_en_ctrl(park_en_ctrl);
}

void ParkCommand::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * ParkCommand::get_data()
{
  return data;
}



// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum104', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void ParkCommand::set_p_check_sum104(int check_sum104) {
  // check_sum104 = ProtocolData::BoundedValue(0, 255, check_sum104);
  int x = check_sum104;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'park_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void ParkCommand::set_p_park_target(bool park_target) {
  int x = park_target;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'park_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void ParkCommand::set_p_park_en_ctrl(bool park_en_ctrl) {
  int x = park_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}


