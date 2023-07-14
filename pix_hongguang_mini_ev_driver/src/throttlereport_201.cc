#include "throttlereport_201.hpp"


Throttlereport201::Throttlereport201() {}
int32_t Throttlereport201::ID = 0x201;

void Throttlereport201::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Throttlereport201::Parse() {
  accpedact = AccPedAct();
  accpedcmd = AccPedCmd();
  accpedexe = AccPedExe();
  accctrlstat = AccCtrlStat();
  accstatcnt = AccStatCnt();
  accstatcks = AccStatCks();
}


// config detail: {'bit': 1, 'description': '实际的加速踏板位置:有效范围0~100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double Throttlereport201::AccPedAct() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 17, 'description': '加速踏板位置指令回传:有效范围0~100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Throttlereport201::AccPedCmd() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 2);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 33, 'description': '执行的加速踏板位置:有效范围0~100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedExe', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double Throttlereport201::AccPedExe() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 2);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 7, 'description': '驱动控制状态:0=禁用,1=使能,2=故障,3保留。', 'is_signed_var': False, 'len': 2, 'name': 'AccCtrlStat', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Throttlereport201::AccCtrlStat() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'AccStatCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Throttlereport201::AccStatCnt() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'AccStatCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Throttlereport201::AccStatCks() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

