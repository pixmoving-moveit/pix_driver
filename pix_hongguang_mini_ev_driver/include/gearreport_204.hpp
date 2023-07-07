#pragma once

#include "Byte.hpp"
#include <iostream>

class Gearreport204 {
public:
    static  int32_t ID;
    Gearreport204();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vehspd;
    int wipercmd;
    int wiperact;
    bool horncmd;
    bool hornstatus;
    int gearact;
    int gearcmd;
    int turnlightcmd;
    int turnlightact;
    int beamlightact;
    int beamlightcmd;
    int gearstatcnt;
    int gearstatcks;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 28, 'description': '车速。', 'is_signed_var': False, 'len': 13, 'name': 'VehSpd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|114.674]', 'physical_unit': 'km/h', 'precision': 0.014, 'type': 'double'}
  double VehSpd();

  // config detail: {'bit': 19, 'description': '雨刮控制指令:0=不开雨刮,1=雨刮低速挡,2=雨刮高速挡,3=雨刮喷水挡。', 'is_signed_var': False, 'len': 2, 'name': 'WiperCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int WiperCmd();

  // config detail: {'bit': 17, 'description': '实际的雨刮状态:0=未开雨刮,1=开启雨刮低速挡,2=开启雨刮高速挡,3=开启雨刮喷水挡。', 'is_signed_var': False, 'len': 2, 'name': 'WiperAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int WiperAct();

  // config detail: {'bit': 23, 'description': '鸣笛控制指令:0:不鸣笛,1:鸣笛。', 'is_signed_var': False, 'len': 1, 'name': 'HornCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
  bool HornCmd();

  // config detail: {'bit': 22, 'description': '鸣笛状态:0=未鸣笛,1=正在鸣笛。', 'is_signed_var': False, 'len': 1, 'name': 'HornStatus', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
  bool HornStatus();

  // config detail: {'bit': 3, 'description': '实际的档位:2=R档,3=N档,4=D档,其他保留。', 'is_signed_var': False, 'len': 4, 'name': 'GearAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|5]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int GearAct();

  // config detail: {'bit': 7, 'description': '档位指令回传:2=R档,3=N档,4=D档,其他保留。', 'is_signed_var': False, 'len': 4, 'name': 'GearCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|5]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int GearCmd();

  // config detail: {'bit': 11, 'description': '转向灯控制指令回传:0=不控制转向灯,1=左转向灯闪烁,2=右转向灯闪烁,3保留。', 'is_signed_var': False, 'len': 2, 'name': 'TurnLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int TurnLightCmd();

  // config detail: {'bit': 9, 'description': '转向灯状态:0=不控制转向灯,1=左转向灯闪烁,2=右转向灯闪烁,3保留。', 'is_signed_var': False, 'len': 2, 'name': 'TurnLightAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int TurnLightAct();

  // config detail: {'bit': 13, 'description': '远近光灯状态:0=灯关闭,1=灯开启,其他:保留。', 'is_signed_var': False, 'len': 2, 'name': 'BeamLightAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int BeamLightAct();

  // config detail: {'bit': 15, 'description': '远近光灯控制指令:0=灯关闭,1=灯开启,其他:保留。', 'is_signed_var': False, 'len': 2, 'name': 'BeamLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int BeamLightCmd();

  // config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'GearStatCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int GearStatCnt();

  // config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'GearStatCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int GearStatCks();
};



