#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class VehicleModeCommand {
public:
	static  int32_t ID;

	VehicleModeCommand();

  	void UpdateData(int check_sum_105, bool headlight_ctrl, int turn_light_ctrl, int drive_mode_ctrl, int steer_mode_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 63, 'description': '校验和', 'is_signed_var': True, 'len': 8, 'name': 'check_sum_105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum_105(int check_sum_105);

  // config detail: {'bit': 18, 'description': '大灯控制', 'is_signed_var': True, 'len': 1, 'name': 'headlight_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_headlight_ctrl(bool headlight_ctrl);

  // config detail: {'bit': 17, 'description': '转向灯控制', 'enum': {0: 'TURNLAMPOFF', 1: 'LEFTTURNLAMPON', 2: 'RIGHTTURNLAMPON', 3: 'HAZARDWARNINGLAMPSTSON'}, 'is_signed_var': True, 'len': 2, 'name': 'turn_light_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_turn_light_ctrl(int turn_light_ctrl);

  // config detail: {'bit': 10, 'description': '驾驶模式控制', 'enum': {0: 'THROTTLE_PADDLE_DRIVE', 1: 'SPEED_DRIVE'}, 'is_signed_var': True, 'len': 3, 'name': 'drive_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_drive_mode_ctrl(int drive_mode_ctrl);

  // config detail: {'bit': 2, 'description': '转向模式控制', 'enum': {0: 'STANDARD_STEER', 1: 'NON_DIRECTION_STEER', 2: 'SYNC_DIRECTION_STEER'}, 'is_signed_var': True, 'len': 3, 'name': 'steer_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_steer_mode_ctrl(int steer_mode_ctrl);

private:
	uint8_t data[8];
};



