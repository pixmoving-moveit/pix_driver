#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class Vcu2AcuSweepSta {
public:
    static const uint32_t ID = 0x519;
    Vcu2AcuSweepSta();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int scu_sweep_conntorller_err2_;
    int scu_sweep_conntorller_err1_;
    int scu_sweep_conntroller_current_;
    int scu_sweep_conntroller_voltage_;
    int scu_sweep_speed_;
    int scu_sweep_travel_mm_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'scu_sweep_conntorller_err2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scusweepconntorllererr2();

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'scu_sweep_conntorller_err1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scusweepconntorllererr1();

  // config detail: {'bit': 32, 'is_signed_var': True, 'len': 16, 'name': 'scu_sweep_conntroller_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scusweepconntrollercurrent();

  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'scu_sweep_conntroller_voltage', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scusweepconntrollervoltage();

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 8, 'name': 'scu_sweep_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scusweepspeed();

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 8, 'name': 'scu_sweep_travel_mm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
  int scusweeptravelmm();
};



