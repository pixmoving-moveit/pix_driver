#pragma once
#include "Byte.hpp"

class Vhiclemodecommand100 {
public:
	static  int32_t ID;

	Vhiclemodecommand100();

  	void UpdateData(bool autoctrlena_, int modectrlcnt_, int modectrlcks_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'description': '自动控制模式总开关:0=禁用,1=使能。', 'is_signed_var': False, 'len': 1, 'name': 'AutoCtrlEna', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
  void set_p_autoctrlena(bool autoctrlena);

  // config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'ModeCtrlCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_modectrlcnt(int modectrlcnt);

  // config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'ModeCtrlCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_modectrlcks(int modectrlcks);

private:
	uint8_t data[8];
};



