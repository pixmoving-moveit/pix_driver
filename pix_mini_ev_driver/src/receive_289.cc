/******************************************************************************
 fulongma
 *****************************************************************************/
#include "receive_289.hpp"


Receive289::Receive289() {}
int32_t Receive289::ID = 0x289;

void Receive289::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Receive289::Parse() {
  ebs_motaglfbk = EBSMotAglFbk();
  ebs_warninglight = EBSWarningLight();
  ebs_stmode = EBSStMode();
  ebs_presfbk = EBSPresFbk();
  ebs_fault_info = EBSFaultInfo();
  ebs_extbrkprio = EBSExtBrkPrio();
  ebs_drvrinteraction = EBSDrvrInteraction();
  ebs_ctroll = EBSctRoll();
  ebs_brkpedalapplied_v = EBSBrkPedalAppliedV();
  ebs_brkpedalapplied = EBSBrkPedalApplied();
  ebs_brakelight = EBSBrakelight();
}


// config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'EBS_MotAglFbk', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Receive289::EBSMotAglFbk() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name': 'EBS_WarningLight', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Receive289::EBSWarningLight() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 4, 'name': 'EBS_StMode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Receive289::EBSStMode() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'EBS_PresFbk', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|125]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Receive289::EBSPresFbk() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name': 'EBS_Fault_Info', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Receive289::EBSFaultInfo() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name': 'EBS_ExtBrkPrio', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Receive289::EBSExtBrkPrio() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name': 'EBS_DrvrInteraction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Receive289::EBSDrvrInteraction() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'is_signed_var': False, 'len': 4, 'name': 'EBS_ctRoll', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Receive289::EBSctRoll() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 1, 'name': 'EBS_BrkPedalApplied_V', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Receive289::EBSBrkPedalAppliedV() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 30, 'is_signed_var': False, 'len': 1, 'name': 'EBS_BrkPedalApplied', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Receive289::EBSBrkPedalApplied() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name': 'EBS_Brakelight', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Receive289::EBSBrakelight() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

