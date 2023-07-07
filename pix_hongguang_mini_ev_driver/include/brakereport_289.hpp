#pragma once

#include "Byte.hpp"
#include <iostream>

class Brakereport289 {
public:
    static  int32_t ID;
    Brakereport289();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int ebs_motaglfbk;
    bool ebs_warninglight;
    int ebs_stmode;
    int ebs_presfbk;
    int ebs_fault_info;
    bool ebs_extbrkprio;
    bool ebs_drvrinteraction;
    int ebs_ctroll;
    bool ebs_brkpedalapplied_v;
    bool ebs_brkpedalapplied;
    bool ebs_brakelight;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'EBS_MotAglFbk', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int EBSMotAglFbk();

  // config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name': 'EBS_WarningLight', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool EBSWarningLight();

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 4, 'name': 'EBS_StMode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int EBSStMode();

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'EBS_PresFbk', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|125]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int EBSPresFbk();

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name': 'EBS_Fault_Info', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int EBSFaultInfo();

  // config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name': 'EBS_ExtBrkPrio', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool EBSExtBrkPrio();

  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name': 'EBS_DrvrInteraction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool EBSDrvrInteraction();

  // config detail: {'bit': 59, 'is_signed_var': False, 'len': 4, 'name': 'EBS_ctRoll', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int EBSctRoll();

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 1, 'name': 'EBS_BrkPedalApplied_V', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool EBSBrkPedalAppliedV();

  // config detail: {'bit': 30, 'is_signed_var': False, 'len': 1, 'name': 'EBS_BrkPedalApplied', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool EBSBrkPedalApplied();

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name': 'EBS_Brakelight', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool EBSBrakelight();
};



