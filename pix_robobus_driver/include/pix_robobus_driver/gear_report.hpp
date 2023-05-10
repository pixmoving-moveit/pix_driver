#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class GearReport {
public:
    static const uint32_t ID = 0x503;
    GearReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int gear_flt_;
    int gear_actual_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'gear_flt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gearflt();

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 3, 'name': 'gear_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|4]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gearactual();
};



