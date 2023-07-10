#include <pix_sweeping_driver/vehicle_mode_command.hpp>

int32_t VehicleModeCommand::ID = 0x105;

// public
VehicleModeCommand::VehicleModeCommand() { Reset(); }

void VehicleModeCommand::UpdateData(int check_sum__105, bool headlight_ctrl, int turn_light_ctrl, int drive_mode_ctrl, int steer_mode_ctrl) {
  set_p_check_sum__105(check_sum__105);
  set_p_headlight_ctrl(headlight_ctrl);
  set_p_turn_light_ctrl(turn_light_ctrl);
  set_p_drive_mode_ctrl(drive_mode_ctrl);
  set_p_steer_mode_ctrl(steer_mode_ctrl);
}

void VehicleModeCommand::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * VehicleModeCommand::get_data()
{
  return data;
}



// config detail: {'bit': 63, 'is_signed_var': True, 'len': 8, 'name': 'check_sum__105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_check_sum__105(int check_sum__105) {
  // check_sum__105 = ProtocolData::BoundedValue(0, 255, check_sum__105);
  int x = check_sum__105;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 18, 'is_signed_var': True, 'len': 1, 'name': 'headlight_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_headlight_ctrl(bool headlight_ctrl) {
  int x = headlight_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 17, 'is_signed_var': True, 'len': 2, 'name': 'turn_light_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_turn_light_ctrl(int turn_light_ctrl) {
  // turn_light_ctrl = ProtocolData::BoundedValue(0, 3, turn_light_ctrl);
  int x = turn_light_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 10, 'is_signed_var': True, 'len': 3, 'name': 'drive_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_drive_mode_ctrl(int drive_mode_ctrl) {
  // drive_mode_ctrl = ProtocolData::BoundedValue(0, 7, drive_mode_ctrl);
  int x = drive_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 2, 'is_signed_var': True, 'len': 3, 'name': 'steer_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_steer_mode_ctrl(int steer_mode_ctrl) {
  // steer_mode_ctrl = ProtocolData::BoundedValue(0, 7, steer_mode_ctrl);
  int x = steer_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[0] += to_set.return_byte_t();
  
}


