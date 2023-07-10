#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class ThrottleCommand {
public:
	static  int32_t ID;

	ThrottleCommand();

  	void UpdateData(int check_sum_100, double dirve_speed_target, double dirve_throttle_pedal_target, bool dirve_en_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum_100', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum_100(int check_sum_100);

  // config detail: {'bit': 39, 'is_signed_var': True, 'len': 12, 'name': 'dirve_speed_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|40.95]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
  void set_p_dirve_speed_target(double dirve_speed_target);

  // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'dirve_throttle_pedal_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_dirve_throttle_pedal_target(double dirve_throttle_pedal_target);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'dirve_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dirve_en_ctrl(bool dirve_en_ctrl);

private:
	uint8_t data[8];
};



