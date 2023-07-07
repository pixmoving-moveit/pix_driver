/******************************************************************************
 fulongma
 *****************************************************************************/

#pragma once

#include "Byte.hpp"
#include <iostream>

class Epsreceive314 {
public:
    static  int32_t ID;
    Epsreceive314();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    bool work_state;
    int tar_angle;
    int cail_sas;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'work_state', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool workstate();

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name': 'tar_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-720|720]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int tarangle();

  // config detail: {'bit': 1, 'enum': {0: 'CAIL_SAS_DEFAULT', 1: 'CAIL_SAS_CAIL'}, 'is_signed_var': False, 'len': 2, 'name': 'cail_sas', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int cailsas();
};



