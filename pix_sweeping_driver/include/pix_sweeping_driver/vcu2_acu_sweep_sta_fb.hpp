#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class Vcu2AcuSweepStaFb {
public:
    static const uint32_t ID = 0x517;
    Vcu2AcuSweepStaFb();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int scu_mowing_speed_fb_;
    int push_rod_travel_difference_alarm_;
    bool end_of_garbage_dumping_des_;
    bool end_of_garbage_dumping_;
    int scu_fan_speed_fb_;
    int scu_liquid_leve_;
    bool scu_heartbeat_;
    bool scan_controller_communication_fault_;
    bool fan_controller_communication_fault_;
    bool scu_filter_clogging_;
    bool scu_sweep_life_end_;
    bool scu_dustbin_full_;
    bool scu_liquid_leve_high_;
    bool scu_liquid_level_low_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 32, 'description': '割草电机实际转速', 'is_signed_var': True, 'len': 16, 'name': 'scu_mowing_speed_fb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int scumowingspeedfb();

  // config detail: {'bit': 56, 'description': '推杆行程差值报警', 'is_signed_var': False, 'len': 2, 'name': 'push_rod_travel_difference_alarm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int pushrodtraveldifferencealarm();

  // config detail: {'bit': 49, 'description': '一键垃圾倾倒下降结束', 'is_signed_var': False, 'len': 1, 'name': 'end_of_garbage_dumping_des', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool endofgarbagedumpingdes();

  // config detail: {'bit': 48, 'description': '垃圾倾倒倾翻结束', 'is_signed_var': False, 'len': 1, 'name': 'end_of_garbage_dumping', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool endofgarbagedumping();

  // config detail: {'bit': 16, 'description': '风机电机实际转速', 'is_signed_var': True, 'len': 16, 'name': 'scu_fan_speed_fb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int scufanspeedfb();

  // config detail: {'bit': 8, 'description': '清水箱液位值', 'is_signed_var': False, 'len': 8, 'name': 'scu_liquid_leve', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int sculiquidleve();

  // config detail: {'bit': 7, 'description': '上装控制器心跳', 'is_signed_var': False, 'len': 1, 'name': 'scu_heartbeat', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scuheartbeat();

  // config detail: {'bit': 6, 'description': '扫盘控制器通信故障报警', 'is_signed_var': False, 'len': 1, 'name': 'scan_controller_communication_fault', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scancontrollercommunicationfault();

  // config detail: {'bit': 5, 'description': '风机控制器通信故障报警', 'is_signed_var': False, 'len': 1, 'name': 'fan_controller_communication_fault', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool fancontrollercommunicationfault();

  // config detail: {'bit': 4, 'description': '滤芯堵赛报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_filter_clogging', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scufilterclogging();

  // config detail: {'bit': 3, 'description': '扫盘磨损结束报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_sweep_life_end', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scusweeplifeend();

  // config detail: {'bit': 2, 'description': '垃圾箱满溢报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_dustbin_full', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scudustbinfull();

  // config detail: {'bit': 1, 'description': '清水箱高液位报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_liquid_leve_high', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool sculiquidlevehigh();

  // config detail: {'bit': 0, 'description': '清水箱低液位报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_liquid_level_low', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool sculiquidlevellow();
};



