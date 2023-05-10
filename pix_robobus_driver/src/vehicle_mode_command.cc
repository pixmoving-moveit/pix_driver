#include <pix_robobus_driver/vehicle_mode_command.hpp>

int32_t VehicleModeCommand::ID = 0x105;

// public
VehicleModeCommand::VehicleModeCommand() { Reset(); }

void VehicleModeCommand::UpdateData(bool auto_prompts, bool clearance_lamp_ctrl, bool turn_right_prompts, bool turn_left_prompts, bool headlight_ctrl, bool back_up_prompts, int vehicle_door_ctrl, int check_sum105, int turn_light_ctrl, bool vehicle_vin_req, int drive_mode_ctrl, int steer_mode_ctrl) {
  set_p_auto_prompts(auto_prompts);
  set_p_clearance_lamp_ctrl(clearance_lamp_ctrl);
  set_p_turn_right_prompts(turn_right_prompts);
  set_p_turn_left_prompts(turn_left_prompts);
  set_p_headlight_ctrl(headlight_ctrl);
  set_p_back_up_prompts(back_up_prompts);
  set_p_vehicle_door_ctrl(vehicle_door_ctrl);
  set_p_check_sum105(check_sum105);
  set_p_turn_light_ctrl(turn_light_ctrl);
  set_p_vehicle_vin_req(vehicle_vin_req);
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



// config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name': 'auto_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_auto_prompts(bool auto_prompts) {
  int x = auto_prompts;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 7, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 19, 'is_signed_var': False, 'len': 1, 'name': 'clearance_lamp_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_clearance_lamp_ctrl(bool clearance_lamp_ctrl) {
  int x = clearance_lamp_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 3, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 20, 'is_signed_var': False, 'len': 1, 'name': 'turn_right_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_turn_right_prompts(bool turn_right_prompts) {
  int x = turn_right_prompts;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 21, 'is_signed_var': False, 'len': 1, 'name': 'turn_left_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_turn_left_prompts(bool turn_left_prompts) {
  int x = turn_left_prompts;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 5, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 18, 'is_signed_var': False, 'len': 1, 'name': 'headlight_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_headlight_ctrl(bool headlight_ctrl) {
  int x = headlight_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 22, 'is_signed_var': False, 'len': 1, 'name': 'back_up_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_back_up_prompts(bool back_up_prompts) {
  int x = back_up_prompts;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 6, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 33, 'is_signed_var': False, 'len': 2, 'name': 'vehicle_door_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_vehicle_door_ctrl(int vehicle_door_ctrl) {
  // vehicle_door_ctrl = ProtocolData::BoundedValue(0, 0, vehicle_door_ctrl);
  int x = vehicle_door_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[4] += to_set.return_byte_t();
  
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_check_sum105(int check_sum105) {
  // check_sum105 = ProtocolData::BoundedValue(0, 255, check_sum105);
  int x = check_sum105;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 17, 'is_signed_var': False, 'len': 2, 'name': 'turn_light_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_turn_light_ctrl(int turn_light_ctrl) {
  // turn_light_ctrl = ProtocolData::BoundedValue(0, 7, turn_light_ctrl);
  int x = turn_light_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name': 'vehicle_vin_req', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void VehicleModeCommand::set_p_vehicle_vin_req(bool vehicle_vin_req) {
  int x = vehicle_vin_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[3] += to_set.return_byte_t();
  
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 3, 'name': 'drive_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_drive_mode_ctrl(int drive_mode_ctrl) {
  // drive_mode_ctrl = ProtocolData::BoundedValue(0, 7, drive_mode_ctrl);
  int x = drive_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 2, 'is_signed_var': False, 'len': 3, 'name': 'steer_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void VehicleModeCommand::set_p_steer_mode_ctrl(int steer_mode_ctrl) {
  // steer_mode_ctrl = ProtocolData::BoundedValue(0, 7, steer_mode_ctrl);
  int x = steer_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[0] += to_set.return_byte_t();
  
}


