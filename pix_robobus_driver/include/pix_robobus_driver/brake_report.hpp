#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class BrakeReport {
public:
    static const uint32_t ID = 0x501;
    BrakeReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double brake_pedal_actual_;
    int brake_flt2_;
    int brake_flt1_;
    int brake_en_state_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'brake_pedal_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double brakepedalactual();

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'brake_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int brakeflt2();

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'brake_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int brakeflt1();

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 2, 'name': 'brake_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int brakeenstate();
};



