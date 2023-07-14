#include "brakecommand_364.hpp"

int32_t Brakecommand364::ID = 0x364;

// public
Brakecommand364::Brakecommand364() { Reset(); }

void Brakecommand364::UpdateData(int vcu_sts_, int vcu_selfstudy_, int vcu_keyst_, bool vcu_extbrkpressure_v_, int vcu_extbrkpressure_, int vcu_ebs_stmodereq_, int vcu_ebs_ctroll_, bool vcu_drvmode_) {
  set_p_vcu_sts(vcu_sts_);
  set_p_vcu_selfstudy(vcu_selfstudy_);
  set_p_vcu_keyst(vcu_keyst_);
  set_p_vcu_extbrkpressure_v(vcu_extbrkpressure_v_);
  set_p_vcu_extbrkpressure(vcu_extbrkpressure_);
  set_p_vcu_ebs_stmodereq(vcu_ebs_stmodereq_);
  set_p_vcu_ebs_ctroll(vcu_ebs_ctroll_);
  set_p_vcu_drvmode(vcu_drvmode_);
}

void Brakecommand364::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Brakecommand364::get_data()
{
  return data;
}



// config detail: {'bit': 29, 'is_signed_var': False, 'len': 2, 'name': 'VCU_sts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Brakecommand364::set_p_vcu_sts(int vcu_sts) {
  // vcu_sts = ProtocolData::BoundedValue(0, 3, vcu_sts);
  int x = vcu_sts;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 2);
  data[3] += to_set.return_byte_t();
  
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'VCU_SelfStudy', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Brakecommand364::set_p_vcu_selfstudy(int vcu_selfstudy) {
  // vcu_selfstudy = ProtocolData::BoundedValue(0, 255, vcu_selfstudy);
  int x = vcu_selfstudy;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 2, 'name': 'VCU_KeySt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Brakecommand364::set_p_vcu_keyst(int vcu_keyst) {
  // vcu_keyst = ProtocolData::BoundedValue(0, 3, vcu_keyst);
  int x = vcu_keyst;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 6, 2);
  data[3] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ExtBrkpressure_V', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Brakecommand364::set_p_vcu_extbrkpressure_v(bool vcu_extbrkpressure_v) {
  int x = vcu_extbrkpressure_v;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'VCU_ExtBrkpressure', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|125]', 'physical_unit': 'Mpa', 'precision': 1.0, 'type': 'int'}
void Brakecommand364::set_p_vcu_extbrkpressure(int vcu_extbrkpressure) {
  // vcu_extbrkpressure = ProtocolData::BoundedValue(0, 125, vcu_extbrkpressure);
  int x = vcu_extbrkpressure;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_StModeReq', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Brakecommand364::set_p_vcu_ebs_stmodereq(int vcu_ebs_stmodereq) {
  // vcu_ebs_stmodereq = ProtocolData::BoundedValue(0, 14, vcu_ebs_stmodereq);
  int x = vcu_ebs_stmodereq;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 4);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 35, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_ctRoll', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Brakecommand364::set_p_vcu_ebs_ctroll(int vcu_ebs_ctroll) {
  // vcu_ebs_ctroll = ProtocolData::BoundedValue(0, 14, vcu_ebs_ctroll);
  int x = vcu_ebs_ctroll;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 4);
  data[4] += to_set.return_byte_t();
  
}

// config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name': 'VCU_DrvMode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Brakecommand364::set_p_vcu_drvmode(bool vcu_drvmode) {
  int x = vcu_drvmode;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 1);
  data[3] += to_set.return_byte_t();
  
}


