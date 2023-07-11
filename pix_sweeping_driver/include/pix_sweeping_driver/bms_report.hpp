#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class BmsReport {
public:
    static const uint32_t ID = 0x512;
    BmsReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double battery_leadacid_voltage_;
    int wireless_charging_sta_;
    int battery_soc_;
    double battery_current_;
    double battery_voltage_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 55, 'description': '铅酸电池电压', 'is_signed_var': True, 'len': 8, 'name': 'battery_leadacid_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|25.4]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double batteryleadacidvoltage();

  // config detail: {'bit': 41, 'description': '无线充电状态', 'is_signed_var': False, 'len': 2, 'name': 'wireless_charging_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int wirelesschargingsta();

  // config detail: {'bit': 39, 'description': '电池SOC', 'is_signed_var': True, 'len': 8, 'name': 'battery_soc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int batterysoc();

  // config detail: {'bit': 23, 'description': '电池电流', 'is_signed_var': True, 'len': 16, 'name': 'battery_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-3200|3353.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double batterycurrent();

  // config detail: {'bit': 7, 'description': '电池电压', 'is_signed_var': True, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
  double batteryvoltage();
};



