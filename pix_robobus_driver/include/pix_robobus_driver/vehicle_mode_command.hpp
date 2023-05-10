#pragma once
#include <pix_robobus_driver/Byte.hpp>

class VehicleModeCommand {
public:
	static  int32_t ID;

	VehicleModeCommand();

  	void UpdateData(bool auto_prompts, bool clearance_lamp_ctrl, bool turn_right_prompts, bool turn_left_prompts, bool headlight_ctrl, bool back_up_prompts, int vehicle_door_ctrl, int check_sum105, int turn_light_ctrl, bool vehicle_vin_req, int drive_mode_ctrl, int steer_mode_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name': 'auto_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_auto_prompts(bool auto_prompts);

  // config detail: {'bit': 19, 'is_signed_var': False, 'len': 1, 'name': 'clearance_lamp_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_clearance_lamp_ctrl(bool clearance_lamp_ctrl);

  // config detail: {'bit': 20, 'is_signed_var': False, 'len': 1, 'name': 'turn_right_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_turn_right_prompts(bool turn_right_prompts);

  // config detail: {'bit': 21, 'is_signed_var': False, 'len': 1, 'name': 'turn_left_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_turn_left_prompts(bool turn_left_prompts);

  // config detail: {'bit': 18, 'is_signed_var': False, 'len': 1, 'name': 'headlight_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_headlight_ctrl(bool headlight_ctrl);

  // config detail: {'bit': 22, 'is_signed_var': False, 'len': 1, 'name': 'back_up_prompts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_back_up_prompts(bool back_up_prompts);

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 2, 'name': 'vehicle_door_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_vehicle_door_ctrl(int vehicle_door_ctrl);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum105(int check_sum105);

  // config detail: {'bit': 17, 'is_signed_var': False, 'len': 2, 'name': 'turn_light_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_turn_light_ctrl(int turn_light_ctrl);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name': 'vehicle_vin_req', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_vehicle_vin_req(bool vehicle_vin_req);

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 3, 'name': 'drive_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_drive_mode_ctrl(int drive_mode_ctrl);

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 3, 'name': 'steer_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_steer_mode_ctrl(int steer_mode_ctrl);

private:
	uint8_t data[8];
};



