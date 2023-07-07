/******************************************************************************
 fulongma
 *****************************************************************************/
#include "sendx_364.hpp"


Sendx364::Sendx364() {}
int32_t Sendx364::ID = 0x364;

void Sendx364::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Sendx364::Parse() {
  vcu_sts = VCUsts();
  vcu_selfstudy = VCUSelfStudy();
  vcu_keyst = VCUKeySt();
  vcu_extbrkpressure_v = VCUExtBrkpressureV();
  vcu_extbrkpressure = VCUExtBrkpressure();
  vcu_ebs_stmodereq = VCUEBSStModeReq();
  vcu_ebs_ctroll = VCUEBSctRoll();
  vcu_drvmode = VCUDrvMode();
}


// config detail: {'bit': 29, 'is_signed_var': False, 'len': 2, 'name': 'VCU_sts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Sendx364::VCUsts() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'VCU_SelfStudy', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Sendx364::VCUSelfStudy() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 2, 'name': 'VCU_KeySt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Sendx364::VCUKeySt() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ExtBrkpressure_V', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Sendx364::VCUExtBrkpressureV() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'VCU_ExtBrkpressure', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|125]', 'physical_unit': 'Mpa', 'precision': 1.0, 'type': 'int'}
int Sendx364::VCUExtBrkpressure() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_StModeReq', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Sendx364::VCUEBSStModeReq() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 35, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_ctRoll', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Sendx364::VCUEBSctRoll() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name': 'VCU_DrvMode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Sendx364::VCUDrvMode() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

