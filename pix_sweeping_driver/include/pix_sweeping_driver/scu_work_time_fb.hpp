#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class ScuWorkTimeFb {
public:
    static const uint32_t ID = 0x51B;
    ScuWorkTimeFb();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double battery_voltage_;
    double work_time_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 8, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|25.4]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double batteryvoltage();

  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 24, 'name': 'work_time', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|838860.7]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double worktime();
};



