#pragma once

#include <pix_hooke_driver/Byte.hpp>
#include <iostream>

class V2achassiswheelrpmfb539 {
public:
    static const uint32_t ID = 0x539;
    V2achassiswheelrpmfb539();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int vcu_chassiswheelrpmlf;
    int vcu_chassiswheelrpmrf;
    int vcu_chassiswheelrpmlr;
    int vcu_chassiswheelrpmrr;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '左前轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmLf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int VCUChassisWheelRpmLf();

  // config detail: {'bit': 16, 'description': '右前轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmRf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int VCUChassisWheelRpmRf();

  // config detail: {'bit': 32, 'description': '左后轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmLr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int VCUChassisWheelRpmLr();

  // config detail: {'bit': 48, 'description': '右后轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmRr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  int VCUChassisWheelRpmRr();
};



