#include "steeringcomand_18f.hpp"

int32_t Steeringcomand18f::ID = 0x18F;

// public
Steeringcomand18f::Steeringcomand18f() { Reset(); }

void Steeringcomand18f::UpdateData(int xjf_, double real_current_, int real_angle_, int ecu_temputer_, bool ecu_state_) {
  set_p_xjf(xjf_);
  set_p_real_current(real_current_);
  set_p_real_angle(real_angle_);
  set_p_ecu_temputer(ecu_temputer_);
  set_p_ecu_state(ecu_state_);
}

void Steeringcomand18f::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Steeringcomand18f::get_data()
{
  return data;
}



// config detail: {'bit': 56, 'is_signed_var': False, 'len': 2, 'name': 'XJF', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringcomand18f::set_p_xjf(int xjf) {
  // xjf = ProtocolData::BoundedValue(0, 3, xjf);
  int x = xjf;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 16, 'name': 'real_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|80]', 'physical_unit': 'A', 'precision': 0.001, 'type': 'double'}
void Steeringcomand18f::set_p_real_current(double real_current) {
  // real_current = ProtocolData::BoundedValue(0.0, 80.0, real_current);
  int x = real_current / 0.001000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[3] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[4] += to_set1.return_byte_t();
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'real_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringcomand18f::set_p_real_angle(int real_angle) {
  // real_angle = ProtocolData::BoundedValue(-720, 720, real_angle);
  int x = real_angle;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[1] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[2] += to_set1.return_byte_t();
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'ECU_temputer', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-40|150]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringcomand18f::set_p_ecu_temputer(int ecu_temputer) {
  // ecu_temputer = ProtocolData::BoundedValue(-40, 150, ecu_temputer);
  int x = ecu_temputer;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[6] += to_set.return_byte_t();
  
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'ECU_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Steeringcomand18f::set_p_ecu_state(bool ecu_state) {
  int x = ecu_state;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}


