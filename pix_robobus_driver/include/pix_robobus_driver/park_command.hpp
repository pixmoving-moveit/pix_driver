#pragma once
#include <pix_robobus_driver/Byte.hpp>

class ParkCommand {
public:
	static  int32_t ID;

	ParkCommand();

  	void UpdateData(int check_sum104, bool park_target, bool park_en_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum104', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum104(int check_sum104);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'park_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_park_target(bool park_target);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'park_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_park_en_ctrl(bool park_en_ctrl);

private:
	uint8_t data[8];
};



