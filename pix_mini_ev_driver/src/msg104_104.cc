/******************************************************************************
fulongma
 *****************************************************************************/
#include "msg104_104.hpp"

int32_t Msg104104::ID = 0x104;

// public
Msg104104::Msg104104() { Reset(); }

void Msg104104::UpdateData(int wipercmd_, bool horncmd_, int gearcmd_, bool gearctrlena_, int turnlightcmd_, int beamlightcmd_, int gearctrlcnt_, int gearctrlcks_) {
  set_p_wipercmd(wipercmd_);
  set_p_horncmd(horncmd_);
  set_p_gearcmd(gearcmd_);
  set_p_gearctrlena(gearctrlena_);
  set_p_turnlightcmd(turnlightcmd_);
  set_p_beamlightcmd(beamlightcmd_);
  set_p_gearctrlcnt(gearctrlcnt_);
  set_p_gearctrlcks(gearctrlcks_);
}

void Msg104104::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Msg104104::get_data()
{
  return data;
}



// config detail: {'bit': 15, 'description': '雨刮控制指令：0=不开雨刮，1=雨刮低速挡，2=雨刮高速挡，3=雨刮喷水挡。', 'is_signed_var': False, 'len': 2, 'name': 'WiperCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Msg104104::set_p_wipercmd(int wipercmd) {
  // wipercmd = ProtocolData::BoundedValue(0, 3, wipercmd);
  int x = wipercmd;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 6, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 12, 'description': '鸣笛控制指令：0：不鸣笛，1：鸣笛。', 'is_signed_var': False, 'len': 1, 'name': 'HornCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
void Msg104104::set_p_horncmd(bool horncmd) {
  int x = horncmd;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 1);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 3, 'description': '档位控制指令：2=R档，3=N档，4=D档，其他保留。', 'is_signed_var': False, 'len': 4, 'name': 'GearCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Msg104104::set_p_gearcmd(int gearcmd) {
  // gearcmd = ProtocolData::BoundedValue(0, 255, gearcmd);
  int x = gearcmd;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 4);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 7, 'description': '档位控制使能：0=禁用，1=使能。', 'is_signed_var': False, 'len': 1, 'name': 'GearCtrlEna', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
void Msg104104::set_p_gearctrlena(bool gearctrlena) {
  int x = gearctrlena;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 7, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 9, 'description': '转向灯控制指令：0=不控制转向灯，1=左转向灯闪烁，2=右转向灯闪烁，3保留。', 'is_signed_var': False, 'len': 2, 'name': 'TurnLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Msg104104::set_p_turnlightcmd(int turnlightcmd) {
  // turnlightcmd = ProtocolData::BoundedValue(0, 3, turnlightcmd);
  int x = turnlightcmd;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 11, 'description': '远近光灯控制指令：0=不开灯，1=开灯，其他：保留。', 'is_signed_var': False, 'len': 2, 'name': 'BeamLightCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Msg104104::set_p_beamlightcmd(int beamlightcmd) {
  // beamlightcmd = ProtocolData::BoundedValue(0, 3, beamlightcmd);
  int x = beamlightcmd;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'GearCtrlCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Msg104104::set_p_gearctrlcnt(int gearctrlcnt) {
  // gearctrlcnt = ProtocolData::BoundedValue(0, 15, gearctrlcnt);
  int x = gearctrlcnt;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 4);
  data[6] += to_set.return_byte_t();
  
}

// config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'GearCtrlCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Msg104104::set_p_gearctrlcks(int gearctrlcks) {
  // gearctrlcks = ProtocolData::BoundedValue(0, 255, gearctrlcks);
  int x = gearctrlcks;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}


