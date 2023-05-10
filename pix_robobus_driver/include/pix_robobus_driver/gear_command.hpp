#pragma once
#include <pix_robobus_driver/Byte.hpp>

class GearCommand {
public:
	static  int32_t ID;

	GearCommand();

  	void UpdateData(int gear_target, bool gear_en_ctrl, int check_sum103);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 3, 'name': 'gear_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|4]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_gear_target(int gear_target);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'gear_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_gear_en_ctrl(bool gear_en_ctrl);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'check_sum103', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_check_sum103(int check_sum103);

private:
	uint8_t data[8];
};



