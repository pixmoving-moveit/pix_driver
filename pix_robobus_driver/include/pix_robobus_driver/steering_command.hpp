#pragma once
#include <pix_robobus_driver/Byte.hpp>

class SteeringCommand {
public:
	static  int32_t ID;

	SteeringCommand();

  	void UpdateData(bool steer_en_ctrl, int steer_angle_target, double steer_angle_speed, int check_sum102);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'steer_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_steer_en_ctrl(bool steer_en_ctrl);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'steer_angle_target', 'offset': -500.0, 'order': 'motorola', 'physical_range': '[-450|450]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  void set_p_steer_angle_target(int steer_angle_target);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'steer_angle_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|500]', 'physical_unit': 'deg/s', 'precision': 2.0, 'type': 'double'}
  void set_p_steer_angle_speed(double steer_angle_speed);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum102', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum102(int check_sum102);

private:
	uint8_t data[8];
};



