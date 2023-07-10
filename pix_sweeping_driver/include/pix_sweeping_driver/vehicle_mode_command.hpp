#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class VehicleModeCommand {
public:
	static  int32_t ID;

	VehicleModeCommand();

  	void UpdateData(int check_sum__105, bool headlight_ctrl, int turn_light_ctrl, int drive_mode_ctrl, int steer_mode_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 63, 'is_signed_var': True, 'len': 8, 'name': 'check_sum__105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum__105(int check_sum__105);

  // config detail: {'bit': 18, 'is_signed_var': True, 'len': 1, 'name': 'headlight_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_headlight_ctrl(bool headlight_ctrl);

  // config detail: {'bit': 17, 'is_signed_var': True, 'len': 2, 'name': 'turn_light_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_turn_light_ctrl(int turn_light_ctrl);

  // config detail: {'bit': 10, 'is_signed_var': True, 'len': 3, 'name': 'drive_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_drive_mode_ctrl(int drive_mode_ctrl);

  // config detail: {'bit': 2, 'is_signed_var': True, 'len': 3, 'name': 'steer_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_steer_mode_ctrl(int steer_mode_ctrl);

private:
	uint8_t data[8];
};



