#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class Vcu2AcuChassisErrCode1 {
public:
    static const uint32_t ID = 0x200;
    Vcu2AcuChassisErrCode1();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    bool bms_communication_fault_;
    bool chassis_emergency_sta_;
    bool leadacid_battery_low_;
    bool controler_communication_fault_;
    bool chassis_crash_sta_;
    bool eps_rear_communication_fault_;
    bool eps_front_communication_fault_;
    bool eds_rr_communication_fault_;
    bool eds_rf_communication_fault_;
    bool eds_lr_communication_fault_;
    bool eds_lf_communication_fault_;
    bool ebs_communication_fault_;
    int chassis_vcu_other_err_;
    int bms_fault1_;
    int bms_fault2_;
    int bms_fault3_;
    int bms_fault4_;
    int bms_fault5_;
    int bms_fault6_;
    int bms_fault7_;
    int bms_fault8_;
    int bms_fault9_;
    int bms_fault10_;
    int bms_fault11_;
    int bms_fault12_;
    int bms_fault13_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 1, 'name': 'bms_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bmscommunicationfault();

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name': 'chassis_emergency_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool chassisemergencysta();

  // config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'leadacid_battery_low', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool leadacidbatterylow();

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'controler_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool controlercommunicationfault();

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'chassis_crash_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool chassiscrashsta();

  // config detail: {'bit': 6, 'is_signed_var': False, 'len': 1, 'name': 'eps_rear_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool epsrearcommunicationfault();

  // config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name': 'eps_front_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool epsfrontcommunicationfault();

  // config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'eds_rr_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool edsrrcommunicationfault();

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'eds_rf_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool edsrfcommunicationfault();

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'eds_lr_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool edslrcommunicationfault();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'eds_lf_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool edslfcommunicationfault();

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'ebs_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool ebscommunicationfault();

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'chassis_vcu_other_err', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisvcuothererr();

  // config detail: {'bit': 26, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault1();

  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault2();

  // config detail: {'bit': 30, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault3();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault4', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault4();

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault5', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault5();

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault6', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault6();

  // config detail: {'bit': 38, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault7', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault7();

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault8', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault8();

  // config detail: {'bit': 42, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault9', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault9();

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault10', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault10();

  // config detail: {'bit': 46, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault11', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault11();

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault12', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault12();

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault13', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bmsfault13();
};



