/******************************************************************************
 fulongma
 *****************************************************************************/

#pragma once

#include "Byte.hpp"
#include <iostream>

class Msg200200 {
public:
    static  int32_t ID;
    Msg200200();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int autoctrlstat;
    int modestatcnt;
    int modestatcks;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 1, 'description': '自动控制模式状态：0=禁用，1=使能，2=故障，3保留。', 'is_signed_var': False, 'len': 2, 'name': 'AutoCtrlStat', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int AutoCtrlStat();

  // config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'ModeStatCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int ModeStatCnt();

  // config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'ModeStatCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int ModeStatCks();
};



