#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class ParkReport {
public:
    static const uint32_t ID = 0x504;
    ParkReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int park_flt_;
    bool parking_actual_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 8, 'name': 'park_flt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int parkflt();

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 1, 'name': 'parking_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool parkingactual();
};


