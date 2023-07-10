#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class Vcu2AcuSweepFanSta {
public:
    static const uint32_t ID = 0x518;
    Vcu2AcuSweepFanSta();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int scu_fan_conntorller_err2_;
    int scu_fan_conntorller_err1_;
    int scu_fan_conntorller_temp_;
    int scu_fan_motor_temp_;
    int scu_fan_conntorller_current_;
    int scu_fan_conntorller_main_voltage_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'scu_fan_conntorller_err2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scufanconntorllererr2();

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'scu_fan_conntorller_err1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scufanconntorllererr1();

  // config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name': 'scu_fan_conntorller_temp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|160]', 'physical_unit': '℃', 'precision': 1.0, 'type': 'int'}
  int scufanconntorllertemp();

  // config detail: {'bit': 32, 'is_signed_var': True, 'len': 8, 'name': 'scu_fan_motor_temp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|210]', 'physical_unit': '℃', 'precision': 1.0, 'type': 'int'}
  int scufanmotortemp();

  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'scu_fan_conntorller_current', 'offset': -2000.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'A', 'precision': 1.0, 'type': 'int'}
  int scufanconntorllercurrent();

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'scu_fan_conntorller_main_voltage', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|500]', 'physical_unit': 'V', 'precision': 1.0, 'type': 'int'}
  int scufanconntorllermainvoltage();
};



