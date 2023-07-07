#pragma once
#include "Byte.hpp"

class Steeringcomand18f {
public:
	static  int32_t ID;

	Steeringcomand18f();

  	void UpdateData(int xjf_, double real_current_, int real_angle_, int ecu_temputer_, bool ecu_state_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 2, 'name': 'XJF', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_xjf(int xjf);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 16, 'name': 'real_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|80]', 'physical_unit': 'A', 'precision': 0.001, 'type': 'double'}
  void set_p_real_current(double real_current);

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'real_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_real_angle(int real_angle);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'ECU_temputer', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-40|150]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_ecu_temputer(int ecu_temputer);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'ECU_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_ecu_state(bool ecu_state);

private:
	uint8_t data[8];
};



