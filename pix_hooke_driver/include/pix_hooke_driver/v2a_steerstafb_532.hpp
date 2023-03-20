// Copyright 2023 Pixmoving, Inc. 
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <pix_hooke_driver/Byte.hpp>
#include <iostream>

class V2asteerstafb532 {
public:
    static const uint32_t ID = 0x532;
    V2asteerstafb532();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int vcu_chassissteerensta;
    int vcu_chassissteerslopover;
    int vcu_chassissteerworkmode;
    int vcu_chassissteermodefb;
    int vcu_chassissteeranglefb;
    int vcu_chassissteeranglerearfb;
    double vcu_chassissteeranglespeedfb;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '转向使能状态', 'enum': {0: 'VCU_CHASSISSTEERENSTA_DISABLE', 1: 'VCU_CHASSISSTEERENSTA_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisSteerEnSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisSteerEnSta();

  // config detail: {'bit': 1, 'description': '转向控制越界提醒', 'enum': {0: 'VCU_CHASSISSTEERSLOPOVER_NORMAL', 1: 'VCU_CHASSISSTEERSLOPOVER_OVER_SLOP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisSteerSlopover', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisSteerSlopover();

  // config detail: {'bit': 2, 'description': '转向线控模式反馈', 'enum': {0: 'VCU_CHASSISSTEERWORKMODE_MACHINE', 1: 'VCU_CHASSISSTEERWORKMODE_WIRE', 2: 'VCU_CHASSISSTEERWORKMODE_POWER'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisSteerWorkMode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisSteerWorkMode();

  // config detail: {'bit': 4, 'description': '转向模式反馈', 'enum': {0: 'VCU_CHASSISSTEERMODEFB_FRONT_ACKERMAN', 1: 'VCU_CHASSISSTEERMODEFB_SAME_FRONT_AND_BACK', 2: 'VCU_CHASSISSTEERMODEFB_FRONT_DIFFERENT_BACK', 3: 'VCU_CHASSISSTEERMODEFB_BACK_ACKRMAN', 4: 'VCU_CHASSISSTEERMODEFB_FRONT_BACK'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_ChassisSteerModeFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisSteerModeFb();

  // config detail: {'bit': 8, 'description': '前转向方向盘转角反馈', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisSteerAngleFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  int VCUChassisSteerAngleFb();

  // config detail: {'bit': 24, 'description': '后转向方向盘转角反馈', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisSteerAngleRearFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  int VCUChassisSteerAngleRearFb();

  // config detail: {'bit': 40, 'description': '设置的转向转角速度反馈', 'is_signed_var': False, 'len': 8, 'name': 'VCU_ChassisSteerAngleSpeedFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|500]', 'physical_unit': 'deg/s', 'precision': 2.0, 'type': 'double'}
  double VCUChassisSteerAngleSpeedFb();
};



