#include "steeringcomand_314.hpp"

int32_t Steeringcomand314::ID = 0x314;

// public
Steeringcomand314::Steeringcomand314() { Reset(); }

void Steeringcomand314::UpdateData(bool work_state_, int tar_angle_, int cail_sas_) {
  set_p_work_state(work_state_);
  set_p_tar_angle(tar_angle_);
  set_p_cail_sas(cail_sas_);
}

void Steeringcomand314::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Steeringcomand314::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'work_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Steeringcomand314::set_p_work_state(bool work_state) {
  int x = work_state;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'tar_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringcomand314::set_p_tar_angle(int tar_angle) {
  // tar_angle = ProtocolData::BoundedValue(-720, 720, tar_angle);
  int x = tar_angle;
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

// config detail: {'bit': 1, 'enum': {0: 'CAIL_SAS_DEFAULT', 1: 'CAIL_SAS_CAIL'}, 'is_signed_var': False, 'len': 2, 'name': 'cail_sas', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Steeringcomand314::set_p_cail_sas(int cail_sas) {
  int x = cail_sas;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 1, 2);
  data[0] += to_set.return_byte_t();
  
}


