/******************************************************************************
 fulongma
 *****************************************************************************/

#pragma once

#include "Byte.hpp"
#include <iostream>

class Epsreceive2315 {
public:
    static  int32_t ID;
    Epsreceive2315();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vech_value;
    bool vech_enable;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'Vech_value', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|250]', 'physical_unit': 'km/h', 'precision': 0.00390625, 'type': 'double'}
  double Vechvalue();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'Vech_enable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool Vechenable();
};



