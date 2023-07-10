#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class Vcu2AcuChassisErrCode2 {
public:
    static const uint32_t ID = 0x201;
    Vcu2AcuChassisErrCode2();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int chassis_ebs_error_code3_;
    int chassis_bms_error_code_;
    int chassis_back_eps_error_code_;
    int chassis_front_eps_error_code_;
    int chassis_ebs_error_code2_;
    int chassis_ebs_error_code1_;
    int chassis_ebs_errorlv_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisebserrorcode3();

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'chassis_bms_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisbmserrorcode();

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'chassis_back_eps_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisbackepserrorcode();

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'chassis_front_eps_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisfrontepserrorcode();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisebserrorcode2();

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisebserrorcode1();

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 4, 'name': 'chassis_ebs_errorlv', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisebserrorlv();
};



