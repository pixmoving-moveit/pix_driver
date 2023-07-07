#pragma once

#include "Byte.hpp"
#include <iostream>

class Steeringreport18f {
public:
    static  int32_t ID;
    Steeringreport18f();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int xjf;
    double real_current;
    int real_angle;
    int ecu_temputer;
    bool ecu_state;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 2, 'name': 'XJF', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int XJF();

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 16, 'name': 'real_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|80]', 'physical_unit': 'A', 'precision': 0.001, 'type': 'double'}
  double realcurrent();

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'real_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int realangle();

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'ECU_temputer', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-40|150]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ECUtemputer();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'ECU_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool ECUstate();
};



