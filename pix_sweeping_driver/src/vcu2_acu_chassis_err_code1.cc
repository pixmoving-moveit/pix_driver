#include <pix_sweeping_driver/vcu2_acu_chassis_err_code1.hpp>


Vcu2AcuChassisErrCode1::Vcu2AcuChassisErrCode1() {}

void Vcu2AcuChassisErrCode1::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuChassisErrCode1::Parse() {
  bms_communication_fault_ = bmscommunicationfault();
  chassis_emergency_sta_ = chassisemergencysta();
  leadacid_battery_low_ = leadacidbatterylow();
  controler_communication_fault_ = controlercommunicationfault();
  chassis_crash_sta_ = chassiscrashsta();
  eps_rear_communication_fault_ = epsrearcommunicationfault();
  eps_front_communication_fault_ = epsfrontcommunicationfault();
  eds_rr_communication_fault_ = edsrrcommunicationfault();
  eds_rf_communication_fault_ = edsrfcommunicationfault();
  eds_lr_communication_fault_ = edslrcommunicationfault();
  eds_lf_communication_fault_ = edslfcommunicationfault();
  ebs_communication_fault_ = ebscommunicationfault();
  chassis_vcu_other_err_ = chassisvcuothererr();
  bms_fault1_ = bmsfault1();
  bms_fault2_ = bmsfault2();
  bms_fault3_ = bmsfault3();
  bms_fault4_ = bmsfault4();
  bms_fault5_ = bmsfault5();
  bms_fault6_ = bmsfault6();
  bms_fault7_ = bmsfault7();
  bms_fault8_ = bmsfault8();
  bms_fault9_ = bmsfault9();
  bms_fault10_ = bmsfault10();
  bms_fault11_ = bmsfault11();
  bms_fault12_ = bmsfault12();
  bms_fault13_ = bmsfault13();
}


// config detail: {'bit': 7, 'is_signed_var': False, 'len': 1, 'name': 'bms_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::bmscommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name': 'chassis_emergency_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::chassisemergencysta() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'leadacid_battery_low', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::leadacidbatterylow() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'controler_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::controlercommunicationfault() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'chassis_crash_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::chassiscrashsta() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 6, 'is_signed_var': False, 'len': 1, 'name': 'eps_rear_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::epsrearcommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name': 'eps_front_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::epsfrontcommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'eds_rr_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::edsrrcommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'eds_rf_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::edsrfcommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'eds_lr_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::edslrcommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'eds_lf_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::edslfcommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'ebs_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuChassisErrCode1::ebscommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'chassis_vcu_other_err', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::chassisvcuothererr() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 26, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault1() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(1, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 28, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault2() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(3, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 30, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault3() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(5, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault4', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault4() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault5', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault5() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(1, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault6', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault6() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(3, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 38, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault7', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault7() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(5, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault8', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault8() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 42, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault9', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault9() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(1, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault10', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault10() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(3, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 46, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault11', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault11() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(5, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault12', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault12() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 7));
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 50, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault13', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode1::bmsfault13() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(1, 2);

  int ret = x;
  return ret;
}

