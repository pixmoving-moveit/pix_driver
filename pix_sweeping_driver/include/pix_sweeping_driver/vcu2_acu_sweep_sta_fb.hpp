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
    bool s_c_u__heartbeat_;
    bool scan_controller_communication_fault_;
    bool fan_controller_communication_fault_;
    bool s_c_u__filter_clogging_;
    bool s_c_u__sweep_life_end_;
    bool s_c_u__dustbin_full_;
    bool scu_liquid_leve_high_;
    bool scu_liquid_level_low_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 32, 'is_signed_var': True, 'len': 16, 'name': 'scu_mowing_speed_fb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int scumowingspeedfb();

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 2, 'name': 'push_rod_travel_difference_alarm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int pushrodtraveldifferencealarm();

  // config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name': 'end_of_garbage_dumping_des', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool endofgarbagedumpingdes();

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name': 'end_of_garbage_dumping', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool endofgarbagedumping();

  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'scu_fan_speed_fb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int scufanspeedfb();

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'scu_liquid_leve', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int sculiquidleve();

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 1, 'name': 's_c_u__heartbeat', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scuheartbeat();

  // config detail: {'bit': 6, 'is_signed_var': False, 'len': 1, 'name': 'scan_controller_communication_fault', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scancontrollercommunicationfault();

  // config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name': 'fan_controller_communication_fault', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool fancontrollercommunicationfault();

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 's_c_u__filter_clogging', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scufilterclogging();

  // config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 's_c_u__sweep_life_end', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scusweeplifeend();

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 's_c_u__dustbin_full', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scudustbinfull();

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'scu_liquid_leve_high', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool sculiquidlevehigh();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'scu_liquid_level_low', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool sculiquidlevellow();
};



