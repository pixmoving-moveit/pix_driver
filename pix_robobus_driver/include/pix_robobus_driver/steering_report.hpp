#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class SteeringReport {
public:
    static const uint32_t ID = 0x502;
    SteeringReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double steer_angle_speed_set_val_;
    int steer_flt2_;
    int steer_flt1_;
    int steer_en_state_;
    int steer_angle_actual_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'steer_angle_speed_set_val', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|500]', 'physical_unit': 'deg/s', 'precision': 2.0, 'type': 'double'}
  double steeranglespeedsetval();

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'steer_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int steerflt2();

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'steer_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int steerflt1();

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 2, 'name': 'steer_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int steerenstate();

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'steer_angle_actual', 'offset': -500.0, 'order': 'motorola', 'physical_range': '[-450|450]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  int steerangleactual();
};



