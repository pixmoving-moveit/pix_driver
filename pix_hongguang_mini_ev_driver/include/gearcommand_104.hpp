#pragma once
#include "Byte.hpp"

class Gearcommand104 {
public:
	static  int32_t ID;

	Gearcommand104();

  	void UpdateData(int wipercmd_, bool horncmd_, int gearcmd_, bool gearctrlena_, int turnlightcmd_, int beamlightcmd_, int gearctrlcnt_, int gearctrlcks_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 15, 'description': '雨刮控制指令:0=不开雨刮,1=雨刮低速挡,2=雨刮高速挡,3=雨刮喷水挡。', 'is_signed_var': False, 'len': 2, 'name': 'WiperCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_wipercmd(int wipercmd);

  // config detail: {'bit': 12, 'description': '鸣笛控制指令:0:不鸣笛,1:鸣笛。', 'is_signed_var': False, 'len': 1, 'name': 'HornCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
  void set_p_horncmd(bool horncmd);

  // config detail: {'bit': 3, 'description': '档位控制指令:2=R档,3=N档,4=D档,其他保留。', 'is_signed_var': False, 'len': 4, 'name': 'GearCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_gearcmd(int gearcmd);

  // config detail: {'bit': 7, 'description': '档位控制使能:0=禁用,1=使能。', 'is_signed_var': False, 'len': 1, 'name': 'GearCtrlEna', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
  void set_p_gearctrlena(bool gearctrlena);

  // config detail: {'bit': 9, 'description': '转向灯控制指令:0=不控制转向灯,1=左转向灯闪烁,2=右转向灯闪烁,3保留。', 'is_signed_var': False, 'len': 2, 'name': 'TurnLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_turnlightcmd(int turnlightcmd);

  // config detail: {'bit': 11, 'description': '远近光灯控制指令:0=不开灯,1=开灯,其他:保留。', 'is_signed_var': False, 'len': 2, 'name': 'BeamLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_beamlightcmd(int beamlightcmd);

  // config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'GearCtrlCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_gearctrlcnt(int gearctrlcnt);

  // config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'GearCtrlCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  void set_p_gearctrlcks(int gearctrlcks);

private:
	uint8_t data[8];
};



