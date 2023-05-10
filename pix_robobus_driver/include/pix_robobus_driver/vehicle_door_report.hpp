#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class VehicleDoorReport {
public:
    static const uint32_t ID = 0x519;
    VehicleDoorReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int door_report_sta_;
    bool door_open_timeout_;
    bool door_open_sta_;
    bool door_open_inplace_;
    bool door_close_timeout_;
    bool door_close_sta_;
    bool door_close_inplace_;
    bool door_button_enable_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 2, 'name': 'door_report_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int doorreportsta();

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'door_open_timeout', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool dooropentimeout();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'door_open_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool dooropensta();

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'door_open_inplace', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool dooropeninplace();

  // config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name': 'door_close_timeout', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool doorclosetimeout();

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'door_close_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool doorclosesta();

  // config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'door_close_inplace', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool doorcloseinplace();

  // config detail: {'bit': 6, 'is_signed_var': False, 'len': 1, 'name': 'door_button_enable', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool doorbuttonenable();
};



