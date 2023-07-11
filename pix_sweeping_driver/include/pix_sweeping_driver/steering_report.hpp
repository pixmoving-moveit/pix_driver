#pragma once

#include <pix_sweeping_driver/Byte.hpp>
#include <iostream>

class SteeringReport {
public:
    static const uint32_t ID = 0x502;
    SteeringReport();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int steer_angle_speed_actual_;
    int steer_angle_rear_actual_;
    int steer_angle_actual_;
    int steer_flt2_;
    int steer_flt1_;
    int steer_en_state_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 63, 'description': '转向实际角速度', 'is_signed_var': True, 'len': 8, 'name': 'steer_angle_speed_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
  int steeranglespeedactual();

  // config detail: {'bit': 47, 'description': '后转向实际角度', 'is_signed_var': True, 'len': 16, 'name': 'steer_angle_rear_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  int steeranglerearactual();

  // config detail: {'bit': 31, 'description': '转向实际角度', 'is_signed_var': True, 'len': 16, 'name': 'steer_angle_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  int steerangleactual();

  // config detail: {'bit': 23, 'description': '转向故障码2', 'is_signed_var': True, 'len': 8, 'name': 'steer_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int steerflt2();

  // config detail: {'bit': 15, 'description': '转向故障码1', 'is_signed_var': True, 'len': 8, 'name': 'steer_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int steerflt1();

  // config detail: {'bit': 1, 'description': '转向使能状态', 'enum': {0: 'MANUAL', 1: 'AUTO', 2: 'TAKEOVER', 3: 'STANDBY'}, 'is_signed_var': True, 'len': 2, 'name': 'steer_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int steerenstate();
};



