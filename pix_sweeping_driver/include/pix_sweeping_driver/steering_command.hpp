#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class SteeringCommand {
public:
	static  int32_t ID;

	SteeringCommand();

  	void UpdateData(int check_sum_102, int steer_angle_target, int steer_angle_speed, bool steer_en_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum_102', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum_102(int check_sum_102);

  // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'steer_angle_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  void set_p_steer_angle_target(int steer_angle_target);

  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 8, 'name': 'steer_angle_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
  void set_p_steer_angle_speed(int steer_angle_speed);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'steer_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_steer_en_ctrl(bool steer_en_ctrl);

private:
	uint8_t data[8];
};



