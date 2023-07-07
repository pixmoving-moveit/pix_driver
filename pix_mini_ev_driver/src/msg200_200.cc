/******************************************************************************
 fulongma
 *****************************************************************************/
#include "msg200_200.hpp"


Msg200200::Msg200200() {}
int32_t Msg200200::ID = 0x200;

void Msg200200::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Msg200200::Parse() {
  autoctrlstat = AutoCtrlStat();
  modestatcnt = ModeStatCnt();
  modestatcks = ModeStatCks();
}


// config detail: {'bit': 1, 'description': '自动控制模式状态：0=禁用，1=使能，2=故障，3保留。', 'is_signed_var': False, 'len': 2, 'name': 'AutoCtrlStat', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg200200::AutoCtrlStat() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'ModeStatCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg200200::ModeStatCnt() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'ModeStatCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
int Msg200200::ModeStatCks() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

