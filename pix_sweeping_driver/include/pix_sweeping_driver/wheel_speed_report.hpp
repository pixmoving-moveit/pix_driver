#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class WheelSpeedReport {
public:
    static const uint32_t ID = 0x506;
    WheelSpeedReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double wheel_speed_r_r_;
    double wheel_speed_r_l_;
    double wheel_speed_f_r_;
    double wheel_speed_f_l_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 55, 'description': '右后轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_r_r', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedrr();

  // config detail: {'bit': 39, 'description': '左后轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_r_l', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedrl();

  // config detail: {'bit': 23, 'description': '右前轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_f_r', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedfr();

  // config detail: {'bit': 7, 'description': '左前轮速', 'is_signed_var': True, 'len': 16, 'name': 'wheel_speed_f_l', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double wheelspeedfl();
};



