/******************************************************************************
fulongma
 *****************************************************************************/
#include "msg100_100.hpp"

int32_t Msg100100::ID = 0x100;

// public
Msg100100::Msg100100() { Reset(); }

void Msg100100::UpdateData(bool autoctrlena_, int modectrlcnt_, int modectrlcks_) {
  set_p_autoctrlena(autoctrlena_);
  set_p_modectrlcnt(modectrlcnt_);
  set_p_modectrlcks(modectrlcks_);
}

void Msg100100::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Msg100100::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'description': '自动控制模式总开关：0=禁用，1=使能。', 'is_signed_var': False, 'len': 1, 'name': 'AutoCtrlEna', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '-', 'precision': 1.0, 'type': 'bool'}
void Msg100100::set_p_autoctrlena(bool autoctrlena) {
  int x = autoctrlena;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'ModeCtrlCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Msg100100::set_p_modectrlcnt(int modectrlcnt) {
  // modectrlcnt = ProtocolData::BoundedValue(0, 15, modectrlcnt);
  int x = modectrlcnt;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 4);
  data[6] += to_set.return_byte_t();
  
}

// config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'ModeCtrlCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
void Msg100100::set_p_modectrlcks(int modectrlcks) {
  // modectrlcks = ProtocolData::BoundedValue(0, 255, modectrlcks);
  int x = modectrlcks;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}


