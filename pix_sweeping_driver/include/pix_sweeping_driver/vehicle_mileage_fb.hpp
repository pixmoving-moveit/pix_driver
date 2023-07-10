#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class VehicleMileageFb {
public:
    static const uint32_t ID = 0x522;
    VehicleMileageFb();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vehicle_sub_mileage_;
    double vehicle_o_d_o_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 39, 'is_signed_var': True, 'len': 24, 'name': 'vehicle_sub_mileage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|167772.15]', 'physical_unit': 'km', 'precision': 0.01, 'type': 'double'}
  double vehiclesubmileage();

  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 24, 'name': 'vehicle_o_d_o', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1677721.5]', 'physical_unit': 'km', 'precision': 0.1, 'type': 'double'}
  double vehicleodo();
};



