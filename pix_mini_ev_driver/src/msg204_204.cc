/******************************************************************************
 fulongma
 *****************************************************************************/
#include "msg204_204.hpp"


Msg204204::Msg204204() {}
int32_t Msg204204::ID = 0x204;

void Msg204204::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Msg204204::Parse() {
  vehspd = VehSpd();
  wipercmd = WiperCmd();
  wiperact = WiperAct();
  horncmd = HornCmd();
  hornstatus = HornStatus();
  gearact = GearAct();
  gearcmd = GearCmd();
  turnlightcmd = TurnLightCmd();
  turnlightact = TurnLightAct();
  beamlightact = BeamLightAct();
  beamlightcmd = BeamLightCmd();
  gearstatcnt = GearStatCnt();
  gearstatcks = GearStatCks();
}


// config detail: {'bit': 28, 'description': '车速。', 'is_signed_var': False, 'len': 13, 'name': 'VehSpd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|114.674]', 'physical_unit': 'km/h', 'precision': 0.014, 'type': 'double'}
double Msg204204::VehSpd() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 5);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.014000;
  return ret;
}

// config detail: {'bit': 19, 'description': '雨刮控制指令：0=不开雨刮，1=雨刮低速挡，2=雨刮高速挡，3=雨刮喷水挡。', 'is_signed_var': False, 'len': 2, 'name': 'WiperCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Msg204204::WiperCmd() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 17, 'description': '实际的雨刮状态：0=未开雨刮，1=开启雨刮低速挡，2=开启雨刮高速挡，3=开启雨刮喷水挡。', 'is_signed_var': False, 'len': 2, 'name': 'WiperAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Msg204204::WiperAct() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'description': '鸣笛控制指令：0：不鸣笛，1：鸣笛。', 'is_signed_var': False, 'len': 1, 'name': 'HornCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
bool Msg204204::HornCmd() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 22, 'description': '鸣笛状态：0=未鸣笛，1=正在鸣笛。', 'is_signed_var': False, 'len': 1, 'name': 'HornStatus', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
bool Msg204204::HornStatus() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 3, 'description': '实际的档位：2=R档，3=N档，4=D档，其他保留。', 'is_signed_var': False, 'len': 4, 'name': 'GearAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|5]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::GearAct() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'description': '档位指令回传：2=R档，3=N档，4=D档，其他保留。', 'is_signed_var': False, 'len': 4, 'name': 'GearCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|5]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::GearCmd() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 11, 'description': '转向灯控制指令回传：0=不控制转向灯，1=左转向灯闪烁，2=右转向灯闪烁，3保留。', 'is_signed_var': False, 'len': 2, 'name': 'TurnLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::TurnLightCmd() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 9, 'description': '转向灯状态：0=不控制转向灯，1=左转向灯闪烁，2=右转向灯闪烁，3保留。', 'is_signed_var': False, 'len': 2, 'name': 'TurnLightAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::TurnLightAct() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 13, 'description': '远近光灯状态：0=灯关闭，1=灯开启，其他：保留。', 'is_signed_var': False, 'len': 2, 'name': 'BeamLightAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::BeamLightAct() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'description': '远近光灯控制指令：0=灯关闭，1=灯开启，其他：保留。', 'is_signed_var': False, 'len': 2, 'name': 'BeamLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::BeamLightCmd() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'GearStatCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::GearStatCnt() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'GearStatCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg204204::GearStatCks() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

