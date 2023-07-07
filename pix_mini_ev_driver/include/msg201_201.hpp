/******************************************************************************
 fulongma
 *****************************************************************************/

#pragma once

#include "Byte.hpp"
#include <iostream>

class Msg201201 {
public:
    static  int32_t ID;
    Msg201201();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double accpedact;
    int accpedcmd;
    double accpedexe;
    int accctrlstat;
    int accstatcnt;
    int accstatcks;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 1, 'description': '实际的加速踏板位置：有效范围0～100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedAct', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double AccPedAct();

  // config detail: {'bit': 17, 'description': '加速踏板位置指令回传：有效范围0～100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedCmd', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int AccPedCmd();

  // config detail: {'bit': 33, 'description': '执行的加速踏板位置：有效范围0～100%。', 'is_signed_var': False, 'len': 10, 'name': 'AccPedExe', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double AccPedExe();

  // config detail: {'bit': 7, 'description': '驱动控制状态：0=禁用，1=使能，2=故障，3保留。', 'is_signed_var': False, 'len': 2, 'name': 'AccCtrlStat', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int AccCtrlStat();

  // config detail: {'bit': 51, 'description': '报文计数器。', 'is_signed_var': False, 'len': 4, 'name': 'AccStatCnt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int AccStatCnt();

  // config detail: {'bit': 63, 'description': '报文校验和。', 'is_signed_var': False, 'len': 8, 'name': 'AccStatCks', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '-', 'precision': 1.0, 'type': 'int'}
  int AccStatCks();
};



