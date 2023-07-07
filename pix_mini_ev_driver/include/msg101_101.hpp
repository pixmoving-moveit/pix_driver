/******************************************************************************
folongma
 *****************************************************************************/

#pragma once
#include "Byte.hpp"

class Msg101101 {
public:
	static  int32_t ID;

	Msg101101();

  	void UpdateData(double accpedcmd_, bool accctrlena_, double accpedinv_, int accctrlcnt_, int accctrlcks_, bool acctkodis_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 1, 'description': '加速踏板位置指令，有效范围0～100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_accpedcmd(double accpedcmd);

  // config detail: {'bit': 7, 'description': '加速踏板控制使能：0=禁用，1=使能。', 'is_signed_var': False, 'len': 1, 'name': 'AccCtrlEna', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
  void set_p_accctrlena(bool accctrlena);

  // config detail: {'bit': 17, 'description': '加速踏板位置校验值：有效范围0～100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedInv', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_accpedinv(double accpedinv);

  // config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'AccCtrlCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_accctrlcnt(int accctrlcnt);

  // config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'AccCtrlCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_accctrlcks(int accctrlcks);

  // config detail: {'bit': 6, 'description': '-', 'is_signed_var': False, 'len': 1, 'name': 'AccTkoDis', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
  void set_p_acctkodis(bool acctkodis);

private:
	uint8_t data[8];
};



