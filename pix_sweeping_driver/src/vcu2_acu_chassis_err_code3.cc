#include <pix_sweeping_driver/vcu2_acu_chassis_err_code3.hpp>


Vcu2AcuChassisErrCode3::Vcu2AcuChassisErrCode3() {}

void Vcu2AcuChassisErrCode3::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuChassisErrCode3::Parse() {
  chassis_rr_eds_error_code2_ = chassisrredserrorcode2();
  chassis_rf_eds_error_code2_ = chassisrfedserrorcode2();
  chassis_lr_eds_error_code2_ = chassislredserrorcode2();
  chassis_lf_eds_error_code2_ = chassislfedserrorcode2();
  chassis_rr_eds_error_code_ = chassisrredserrorcode();
  chassis_rf_eds_error_code_ = chassisrfedserrorcode();
  chassis_lr_eds_error_code_ = chassislredserrorcode();
  chassis_lf_eds_error_code_ = chassislfedserrorcode();
}


// config detail: {'bit': 56, 'description': '右后电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rr_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassisrredserrorcode2() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 8));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 40, 'description': '右前电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rf_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassisrfedserrorcode2() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 24, 'description': '左后电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lr_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassislredserrorcode2() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '左前电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lf_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassislfedserrorcode2() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 48, 'description': '右后电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rr_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassisrredserrorcode() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 7));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 32, 'description': '右前电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rf_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassisrfedserrorcode() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 16, 'description': '左后电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lr_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassislredserrorcode() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 0, 'description': '左前电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lf_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode3::chassislfedserrorcode() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

