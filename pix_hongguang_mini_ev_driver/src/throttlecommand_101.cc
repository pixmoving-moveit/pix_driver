#include "throttlecommand_101.hpp"

int32_t Throttlecommand101::ID = 0x101;

// public
Throttlecommand101::Throttlecommand101() { Reset(); }

void Throttlecommand101::UpdateData(double accpedcmd_, bool accctrlena_, double accpedinv_, int accctrlcnt_, int accctrlcks_, bool acctkodis_) {
  set_p_accpedcmd(accpedcmd_);
  set_p_accctrlena(accctrlena_);
  set_p_accpedinv(accpedinv_);
  set_p_accctrlcnt(accctrlcnt_);
  set_p_accctrlcks(accctrlcks_);
  set_p_acctkodis(acctkodis_);
}

void Throttlecommand101::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Throttlecommand101::get_data()
{
  return data;
}



// config detail: {'bit': 1, 'description': '加速踏板位置指令,有效范围0~100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void Throttlecommand101::set_p_accpedcmd(double accpedcmd) {
  // accpedcmd = ProtocolData::BoundedValue(0.0, 100.0, accpedcmd);
  int x = accpedcmd / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[1] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0x3;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 2);
  data[0] += to_set1.return_byte_t();
}

// config detail: {'bit': 7, 'description': '加速踏板控制使能:0=禁用,1=使能。', 'is_signed_var': False, 'len': 1, 'name': 'AccCtrlEna', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
void Throttlecommand101::set_p_accctrlena(bool accctrlena) {
  int x = accctrlena;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 7, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 17, 'description': '加速踏板位置校验值:有效范围0~100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedInv', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void Throttlecommand101::set_p_accpedinv(double accpedinv) {
  // accpedinv = ProtocolData::BoundedValue(0.0, 100.0, accpedinv);
  int x = accpedinv / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[3] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0x3;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 2);
  data[2] += to_set1.return_byte_t();
}

// config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'AccCtrlCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Throttlecommand101::set_p_accctrlcnt(int accctrlcnt) {
  // accctrlcnt = ProtocolData::BoundedValue(0, 15, accctrlcnt);
  int x = accctrlcnt;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 4);
  data[6] += to_set.return_byte_t();
  
}

// config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'AccCtrlCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Throttlecommand101::set_p_accctrlcks(int accctrlcks) {
  // accctrlcks = ProtocolData::BoundedValue(0, 255, accctrlcks);
  int x = accctrlcks;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 6, 'description': '-', 'is_signed_var': False, 'len': 1, 'name': 'AccTkoDis', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
void Throttlecommand101::set_p_acctkodis(bool acctkodis) {
  int x = acctkodis;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 6, 1);
  data[0] += to_set.return_byte_t();
  
}


