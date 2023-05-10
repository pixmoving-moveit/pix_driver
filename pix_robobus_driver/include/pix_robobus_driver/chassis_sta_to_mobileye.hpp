#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class ChassisStaToMobileye {
public:
    static const uint32_t ID = 0x150;
    ChassisStaToMobileye();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vehicle_speed_to_mbe_;
    bool brake_light_actual_;
    int turn_light_actual_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'vehicle_speed_to_mbe', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'kph', 'precision': 0.1, 'type': 'double'}
  double vehiclespeedtombe();

  // config detail: {'bit': 18, 'is_signed_var': False, 'len': 1, 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool brakelightactual();

  // config detail: {'bit': 17, 'is_signed_var': False, 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int turnlightactual();
};



