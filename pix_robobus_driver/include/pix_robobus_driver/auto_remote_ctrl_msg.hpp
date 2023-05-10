#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class AutoRemoteCtrlMsg {
public:
    static const uint32_t ID = 0x3B0;
    AutoRemoteCtrlMsg();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int auto_remote_heartbeat_;
    int auto_remote_drive_ctrl_mode_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'auto_remote_heartbeat', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int autoremoteheartbeat();

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'auto_remote_drive_ctrl_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int autoremotedrivectrlmode();
};



