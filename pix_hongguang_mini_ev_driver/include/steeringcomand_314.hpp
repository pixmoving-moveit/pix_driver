#pragma once
#include "Byte.hpp"

class Steeringcomand314 {
public:
	static  int32_t ID;

	Steeringcomand314();

  	void UpdateData(bool work_state_, int tar_angle_, int cail_sas_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'work_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_work_state(bool work_state);

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'tar_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_tar_angle(int tar_angle);

  // config detail: {'bit': 1, 'enum': {0: 'CAIL_SAS_DEFAULT', 1: 'CAIL_SAS_CAIL'}, 'is_signed_var': False, 'len': 2, 'name': 'cail_sas', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_cail_sas(int cail_sas);

private:
	uint8_t data[8];
};



