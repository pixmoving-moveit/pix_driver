#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class BrakeCommand {
public:
	static  int32_t ID;

	BrakeCommand();

  	void UpdateData(int check_sum_101, double brake_pedal_target, bool brake_en_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum_101', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum_101(int check_sum_101);

  // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'brake_pedal_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_brake_pedal_target(double brake_pedal_target);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'brake_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_brake_en_ctrl(bool brake_en_ctrl);

private:
	uint8_t data[8];
};



