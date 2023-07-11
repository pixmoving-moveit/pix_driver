#include <pix_sweeping_driver/vehicle_mode_command.hpp>

int32_t VehicleModeCommand::ID = 0x105;

// public
VehicleModeCommand::VehicleModeCommand() { Reset(); }

void VehicleModeCommand::UpdateData(int check_sum_105, bool headlight_ctrl, int turn_light_ctrl, int drive_mode_ctrl, int steer_mode_ctrl) {
  set_p_check_sum_105(check_sum_105);
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



// config detail: {'bit': 63, 'description': '校验和', 'is_signed_var': True, 'len': 8, 'name': 'check_sum_105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_check_sum_105(int check_sum_105) {
  // check_sum_105 = ProtocolData::BoundedValue(0, 255, check_sum_105);
  int x = check_sum_105;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 18, 'description': '大灯控制', 'is_signed_var': True, 'len': 1, 'name': 'headlight_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_headlight_ctrl(bool headlight_ctrl) {
  int x = headlight_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 17, 'description': '转向灯控制', 'enum': {0: 'TURNLAMPOFF', 1: 'LEFTTURNLAMPON', 2: 'RIGHTTURNLAMPON', 3: 'HAZARDWARNINGLAMPSTSON'}, 'is_signed_var': True, 'len': 2, 'name': 'turn_light_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void VehicleModeCommand::set_p_turn_light_ctrl(int turn_light_ctrl) {
  int x = turn_light_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 10, 'description': '驾驶模式控制', 'enum': {0: 'THROTTLE_PADDLE_DRIVE', 1: 'SPEED_DRIVE'}, 'is_signed_var': True, 'len': 3, 'name': 'drive_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void VehicleModeCommand::set_p_drive_mode_ctrl(int drive_mode_ctrl) {
  int x = drive_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 2, 'description': '转向模式控制', 'enum': {0: 'STANDARD_STEER', 1: 'NON_DIRECTION_STEER', 2: 'SYNC_DIRECTION_STEER'}, 'is_signed_var': True, 'len': 3, 'name': 'steer_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void VehicleModeCommand::set_p_steer_mode_ctrl(int steer_mode_ctrl) {
  int x = steer_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[0] += to_set.return_byte_t();
  
}


