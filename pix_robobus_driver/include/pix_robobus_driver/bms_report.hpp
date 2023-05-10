#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class BmsReport {
public:
    static const uint32_t ID = 0x512;
    BmsReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double battery_leadacid_voltage_;
    double battery_current_;
    double battery_voltage_;
    int battery_soc_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'battery_leadacid_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|25.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double batteryleadacidvoltage();

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'battery_current', 'offset': -3200.0, 'order': 'motorola', 'physical_range': '[-3200|3353.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double batterycurrent();

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
  double batteryvoltage();

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 8, 'name': 'battery_soc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int batterysoc();
};



