#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class ThrottleReport {
public:
    static const uint32_t ID = 0x500;
    ThrottleReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double dirve_throttle_pedal_actual_;
    int dirve_flt2_;
    int dirve_flt1_;
    int dirve_en_state_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'dirve_throttle_pedal_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double dirvethrottlepedalactual();

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'dirve_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int dirveflt2();

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'dirve_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int dirveflt1();

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 2, 'name': 'dirve_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int dirveenstate();
};



