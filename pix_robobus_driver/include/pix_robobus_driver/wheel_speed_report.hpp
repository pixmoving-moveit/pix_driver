#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class WheelSpeedReport {
public:
    static const uint32_t ID = 0x506;
    WheelSpeedReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double wheel_speed_rr_;
    double wheel_speed_rl_;
    double wheel_speed_fr_;
    double wheel_speed_fl_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_rr', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedrr();

  // config detail: {'bit': 39, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_rl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedrl();

  // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_fr', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedfr();

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_fl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedfl();
};



