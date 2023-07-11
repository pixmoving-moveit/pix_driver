#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class BrakeCommand {
public:
	static  int32_t ID;

	BrakeCommand();

  	void UpdateData(int check_sum101, double brake_pedal_target, bool brake_en_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 63, 'description': '校验和', 'is_signed_var': False, 'len': 8, 'name': 'check_sum101', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum101(int check_sum101);

  // config detail: {'bit': 23, 'description': '制动油门', 'is_signed_var': True, 'len': 16, 'name': 'brake_pedal_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_brake_pedal_target(double brake_pedal_target);

  // config detail: {'bit': 0, 'description': '制动使能', 'is_signed_var': False, 'len': 1, 'name': 'brake_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_brake_en_ctrl(bool brake_en_ctrl);

private:
	uint8_t data[8];
};



