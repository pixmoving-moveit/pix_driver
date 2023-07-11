#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class Vcu2AcuSweepWorkSta {
public:
    static const uint32_t ID = 0x521;
    Vcu2AcuSweepWorkSta();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    bool sweep_emergency_sig_fb_;
    int vcu_sweep_plate_up_down_sta_fb_;
    bool vcu_auto_garbage_dump_sta_fb_;
    bool vcu_auto_cleaning_sta_fb_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 10, 'description': '清扫系统紧急停止反馈', 'is_signed_var': True, 'len': 1, 'name': 'sweep_emergency_sig_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool sweepemergencysigfb();

  // config detail: {'bit': 9, 'description': '扫盘升降状态反馈', 'enum': {0: 'NO', 1: 'UP', 2: 'DOWN'}, 'is_signed_var': True, 'len': 2, 'name': 'vcu_sweep_plate_up_down_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int vcusweepplateupdownstafb();

  // config detail: {'bit': 1, 'description': '一键倾倒状态反馈', 'is_signed_var': True, 'len': 1, 'name': 'vcu_auto_garbage_dump_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool vcuautogarbagedumpstafb();

  // config detail: {'bit': 0, 'description': '一键清扫状态反馈', 'is_signed_var': True, 'len': 1, 'name': 'vcu_auto_cleaning_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool vcuautocleaningstafb();
};



