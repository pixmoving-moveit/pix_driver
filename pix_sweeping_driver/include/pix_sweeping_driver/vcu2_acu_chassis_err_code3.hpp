#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class Vcu2AcuChassisErrCode3 {
public:
    static const uint32_t ID = 0x202;
    Vcu2AcuChassisErrCode3();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int chassis_rr_eds_error_code2_;
    int chassis_rf_eds_error_code2_;
    int chassis_lr_eds_error_code2_;
    int chassis_lf_eds_error_code2_;
    int chassis_rr_eds_error_code_;
    int chassis_rf_eds_error_code_;
    int chassis_lr_eds_error_code_;
    int chassis_lf_eds_error_code_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 56, 'description': '右后电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rr_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassisrredserrorcode2();

  // config detail: {'bit': 40, 'description': '右前电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rf_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassisrfedserrorcode2();

  // config detail: {'bit': 24, 'description': '左后电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lr_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassislredserrorcode2();

  // config detail: {'bit': 8, 'description': '左前电控故障2', 'enum': {1: 'MCU_CHIP_ERROR', 2: 'PRE_CHARGE_ERROR', 4: 'MOSTFET_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lf_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassislfedserrorcode2();

  // config detail: {'bit': 48, 'description': '右后电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rr_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassisrredserrorcode();

  // config detail: {'bit': 32, 'description': '右前电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rf_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassisrfedserrorcode();

  // config detail: {'bit': 16, 'description': '左后电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lr_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassislredserrorcode();

  // config detail: {'bit': 0, 'description': '左前电控故障', 'enum': {1: 'OVER_CURRENT', 2: 'UNDER_VOLTAGE', 4: 'HALL_SENSOR_ERROR', 8: 'OVER_VOLTAGE', 16: 'MOTOR_BLOCK', 32: 'SPEED_FEEDBACK', 64: 'MCU_EEPROM_WRITE_ERROR', 128: 'OVERHEAT'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lf_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassislfedserrorcode();
};



