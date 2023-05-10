#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class VehicleMileageFb {
public:
    static const uint32_t ID = 0x522;
    VehicleMileageFb();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vehicle_sub_mileage_;
    double vehicle_odo_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 24, 'name': 'vehicle_sub_mileage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|16777.215]', 'physical_unit': 'km', 'precision': 0.001, 'type': 'double'}
  double vehiclesubmileage();

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 24, 'name': 'vehicle_odo', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1677721.5]', 'physical_unit': 'km', 'precision': 0.1, 'type': 'double'}
  double vehicleodo();
};



