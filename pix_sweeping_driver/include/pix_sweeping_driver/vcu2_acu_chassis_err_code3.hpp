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
    
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rr_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisrredserrorcode2();

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rf_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisrfedserrorcode2();

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lr_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassislredserrorcode2();

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lf_eds_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassislfedserrorcode2();

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rr_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisrredserrorcode();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'chassis_rf_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisrfedserrorcode();

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lr_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassislredserrorcode();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'chassis_lf_eds_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassislfedserrorcode();
};



