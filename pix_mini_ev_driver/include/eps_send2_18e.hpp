/******************************************************************************
 fulongma
 *****************************************************************************/

#pragma once

#include "Byte.hpp"
#include <iostream>

class Epssend218e {
public:
    static  int32_t ID;
    Epssend218e();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    bool work_mode;
    double torger_value;
    double tar_current;
    double steer_speed;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'work_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool workmode();

  // config detail: {'bit': 40, 'is_signed_var': True, 'len': 16, 'name': 'Torger_value', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-6|6]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  double Torgervalue();

  // config detail: {'bit': 24, 'is_signed_var': True, 'len': 16, 'name': 'Tar_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-40|40]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double Tarcurrent();

  // config detail: {'bit': 56, 'is_signed_var': True, 'len': 16, 'name': 'Steer_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-4096|4096]', 'physical_unit': '\u3000deg/s', 'precision': 0.1, 'type': 'double'}
  double Steerspeed();
};



