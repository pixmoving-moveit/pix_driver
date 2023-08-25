#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class AutoCtrlMsg {
    public:
        static const uint32_t ID = 0x3A0;
        AutoCtrlMsg();
        void Parse();
        void update_bytes(uint8_t bytes_data[8]);
        // singal
        int auto_heartbeat_;
        int auto_drive_ctrl_mode_;
        uint8_t count;

        void set_auto_drive_ctrl_mode(int mode);
        void set_hearbeat();

        uint8_t *get_data();
        void Reset();

    private:
        uint8_t bytes[8];
        
        // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'auto_heartbeat', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
        int autoheartbeat();

        // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'auto_drive_ctrl_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
        int autodrivectrlmode();
};



