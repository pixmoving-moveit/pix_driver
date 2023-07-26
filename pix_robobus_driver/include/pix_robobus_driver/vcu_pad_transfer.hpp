#pragma once

#include <pix_robobus_driver/Byte.hpp>
#include <iostream>

class VCUPadTransfer {
public:
    static const uint32_t ID = 0x53B;
    VCUPadTransfer();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    bool v_c_u__pad_auto_start_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'v_c_u__pad_auto_start', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool vcupadautostart();
};



