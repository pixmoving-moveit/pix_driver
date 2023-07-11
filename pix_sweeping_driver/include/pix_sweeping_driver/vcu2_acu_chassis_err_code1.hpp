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
    int bms_communication_fault_;
    int chassis_emergency_sta_;
    int leadacid_battery_low_;
    int controler_communication_fault_;
    int chassis_crash_sta_;
    int eps_rear_communication_fault_;
    int eps_front_communication_fault_;
    int eds_rr_communication_fault_;
    int eds_rf_communication_fault_;
    int eds_lr_communication_fault_;
    int eds_lf_communication_fault_;
    int ebs_communication_fault_;
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
    
  // config detail: {'bit': 7, 'description': 'BMS通讯故', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'bms_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmscommunicationfault();

  // config detail: {'bit': 10, 'description': '底盘急停', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'chassis_emergency_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassisemergencysta();

  // config detail: {'bit': 11, 'description': '铅酸电池电量低', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'leadacid_battery_low', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int leadacidbatterylow();

  // config detail: {'bit': 8, 'description': '控制器通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'controler_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int controlercommunicationfault();

  // config detail: {'bit': 9, 'description': '防撞条触发状态', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'chassis_crash_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassiscrashsta();

  // config detail: {'bit': 6, 'description': '后EPS通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'eps_rear_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int epsrearcommunicationfault();

  // config detail: {'bit': 5, 'description': '前EPS通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'eps_front_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int epsfrontcommunicationfault();

  // config detail: {'bit': 3, 'description': '右后电控通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'eds_rr_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int edsrrcommunicationfault();

  // config detail: {'bit': 2, 'description': '右前电控通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'eds_rf_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int edsrfcommunicationfault();

  // config detail: {'bit': 1, 'description': '左后电控通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'eds_lr_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int edslrcommunicationfault();

  // config detail: {'bit': 0, 'description': '左前电控通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'eds_lf_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int edslfcommunicationfault();

  // config detail: {'bit': 4, 'description': 'EHB通讯故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 1, 'name': 'ebs_communication_fault', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int ebscommunicationfault();

  // config detail: {'bit': 16, 'description': 'VCU其他故障', 'enum': {0: 'TRUE', 1: 'FALSE'}, 'is_signed_var': False, 'len': 8, 'name': 'chassis_vcu_other_err', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int chassisvcuothererr();

  // config detail: {'bit': 26, 'description': '单体过压故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault1();

  // config detail: {'bit': 28, 'description': '单体低压故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault2();

  // config detail: {'bit': 30, 'description': '单体压差过大故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault3();

  // config detail: {'bit': 32, 'description': '放电高温故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault4', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault4();

  // config detail: {'bit': 34, 'description': '放电低温故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault5', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault5();

  // config detail: {'bit': 36, 'description': '充电高温故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault6', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault6();

  // config detail: {'bit': 38, 'description': '充电低温故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault7', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault7();

  // config detail: {'bit': 40, 'description': '单体温差过大故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault8', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault8();

  // config detail: {'bit': 42, 'description': '总压过低故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault9', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault9();

  // config detail: {'bit': 44, 'description': '充电过流故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault10', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault10();

  // config detail: {'bit': 46, 'description': '放电过流故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault11', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault11();

  // config detail: {'bit': 48, 'description': 'SOC过低故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault12', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault12();

  // config detail: {'bit': 50, 'description': '总压过低故障', 'enum': {0: 'NORMAL', 1: 'LEVEL_1_ALARM', 2: 'LEVEL_2_ALARM', 3: 'LEVEL_3_ALARM'}, 'is_signed_var': False, 'len': 2, 'name': 'bms_fault13', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|17]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int bmsfault13();
};



