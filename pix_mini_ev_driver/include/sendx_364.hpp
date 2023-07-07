/******************************************************************************
 fulongma
 *****************************************************************************/

#pragma once

#include "Byte.hpp"
#include <iostream>

class Sendx364 {
public:
    static  int32_t ID;
    Sendx364();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int vcu_sts;
    int vcu_selfstudy;
    int vcu_keyst;
    bool vcu_extbrkpressure_v;
    int vcu_extbrkpressure;
    int vcu_ebs_stmodereq;
    int vcu_ebs_ctroll;
    bool vcu_drvmode;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 29, 'is_signed_var': False, 'len': 2, 'name': 'VCU_sts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int VCUsts();

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'VCU_SelfStudy', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int VCUSelfStudy();

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 2, 'name': 'VCU_KeySt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int VCUKeySt();

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ExtBrkpressure_V', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool VCUExtBrkpressureV();

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'VCU_ExtBrkpressure', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|125]', 'physical_unit': 'Mpa', 'precision': 1.0, 'type': 'int'}
  int VCUExtBrkpressure();

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_StModeReq', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int VCUEBSStModeReq();

  // config detail: {'bit': 35, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_ctRoll', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int VCUEBSctRoll();

  // config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name': 'VCU_DrvMode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool VCUDrvMode();
};



