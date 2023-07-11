#include <pix_sweeping_driver/vcu2_acu_chassis_err_code2.hpp>


Vcu2AcuChassisErrCode2::Vcu2AcuChassisErrCode2() {}

void Vcu2AcuChassisErrCode2::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuChassisErrCode2::Parse() {
  chassis_ebs_error_code3_ = chassisebserrorcode3();
  chassis_bms_error_code_ = chassisbmserrorcode();
  chassis_back_eps_error_code_ = chassisbackepserrorcode();
  chassis_front_eps_error_code_ = chassisfrontepserrorcode();
  chassis_ebs_error_code2_ = chassisebserrorcode2();
  chassis_ebs_error_code1_ = chassisebserrorcode1();
  chassis_ebs_errorlv_ = chassisebserrorlv();
}


// config detail: {'bit': 40, 'description': '制动故障码3', 'enum': {0: 'CURRENT_SENSOR_MALFUNCTION', 1: 'LOW_POWER_SUPPLY_VOLTAGE_FAULT', 2: 'HIGH_POWER_SUPPLY_VOLTAGE_FAULT', 4: 'POWER_SWITCH_FAULT', 8: 'PRE_DRIVE_FAULT', 16: 'EHB_MOTOR_POSITION_FAILURE'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode2::chassisebserrorcode3() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 16, 'description': '', 'is_signed_var': False, 'len': 8, 'name': 'chassis_bms_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisbmserrorcode() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'description': '后转向故障码', 'enum': {1: 'BATTERY_UNDERVOLTAGE', 2: 'BATTERY_OVERVOLTAGE', 4: 'OVERTEMPERATURE', 8: 'OVERCURRENT', 16: 'EEPROM_COMMUNICATION', 32: 'CURRENT_TRACKING', 64: 'CURRENT_SENSOR', 128: 'OTHER_FAULTS'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_back_eps_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode2::chassisbackepserrorcode() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 8));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 48, 'description': '前转向故障码', 'enum': {1: 'BATTERY_UNDERVOLTAGE', 2: 'BATTERY_OVERVOLTAGE', 4: 'OVERTEMPERATURE', 8: 'OVERCURRENT', 16: 'EEPROM_COMMUNICATION', 32: 'CURRENT_TRACKING', 64: 'CURRENT_SENSOR', 128: 'OTHER_FAULTS'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_front_eps_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode2::chassisfrontepserrorcode() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 7));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 32, 'description': '制动故障码2', 'enum': {1: 'ABNORMAL_LOW_POWER_SUPPLY_VOLTAGE', 2: 'ABNORMAL_HIGH_POWER_SUPPLY_VOLTAGE', 4: 'SEVERE_ABNORMALITY_IN_EHB_HYDRAULIC_PRESSURE_TRACKING', 8: 'SEVERE_OSCILLATION_IN_EHB_HYDRAULIC_PRESSURE', 16: 'LOAD_MISMATCH_FAULT', 32: 'CAN_COMMUNICATION_FAULT', 64: 'NOT_USED', 128: 'EHB_OVERCURRENT_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode2::chassisebserrorcode2() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 24, 'description': '制动故障码1', 'enum': {1: 'ABNORMAL_BRAKE_COMMAND', 2: 'SLIGHT_DEVIATION_IN_EHB_HYDRAULIC_PRESSURE_TRACKING', 4: 'SLIGHT_OSCILLATION_IN_EHB_HYDRAULIC_PRESSURE', 8: 'PARTIAL_TEMPERATURE_SENSOR_MALFUNCTION', 16: 'NOT_USED', 32: 'COMPLETE_FAILURE_OF_TEMPERATURE_SENSOR', 64: 'HYDRAULIC_PRESSURE_SENSOR_MALFUNCTION', 128: 'CONTROLLER_TEMPERATURE_TOO_HIGH'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode2::chassisebserrorcode1() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 16, 'description': '制动故障级别', 'enum': {0: 'NO_FAULT', 1: 'LEVEL_1_FAULT', 2: 'LEVEL_2_FAULT', 3: 'LEVEL_3_FAULT', 4: 'LEVEL_4_FAULT'}, 'is_signed_var': False, 'len': 4, 'name': 'chassis_ebs_errorlv', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuChassisErrCode2::chassisebserrorlv() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

