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

class V2adrivestafb530 {
public:
  static const uint32_t ID = 0x530;
  V2adrivestafb530();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  // singal
  int vcu_chassisdriverensta;
  int vcu_chassisdiverslopover;
  int vcu_chassisdrivermodesta;
  int vcu_chassisgearfb;
  double vcu_chassisspeedfb;
  double vcu_chassisthrottlepaldfb;
  double vcu_chassisaccelerationfb;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '驱动使能状态', 'enum': {0: 'VCU_CHASSISDRIVERENSTA_DISABLE', 1: 'VCU_CHASSISDRIVERENSTA_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisDriverEnSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisDriverEnSta();

  // config detail: {'bit': 1, 'description': '驱动控制越界提醒', 'enum': {0: 'VCU_CHASSISDIVERSLOPOVER_NORMAL', 1: 'VCU_CHASSISDIVERSLOPOVER_OVER_SLOP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisDiverSlopover', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisDiverSlopover();

  // config detail: {'bit': 2, 'description': '驱动模式反馈', 'enum': {0: 'VCU_CHASSISDRIVERMODESTA_SPEED_CTRL_MODE', 1: 'VCU_CHASSISDRIVERMODESTA_THROTTLE_CTRL_MODE', 2: 'VCU_CHASSISDRIVERMODESTA_RESERVE', 3: 'VCU_CHASSISDRIVERMODESTA_RESERVE'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisDriverModeSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisDriverModeSta();

  // config detail: {'bit': 4, 'description': '档位状态', 'enum': {0: 'VCU_CHASSISGEARFB_NO_USE', 1: 'VCU_CHASSISGEARFB_D', 2: 'VCU_CHASSISGEARFB_N', 3: 'VCU_CHASSISGEARFB_R'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisGearFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisGearFb();

  // config detail: {'bit': 8, 'description': '车速实际反馈', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisSpeedFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-50|50]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
  double VCUChassisSpeedFb();

  // config detail: {'bit': 24, 'description': '油门请求值反馈', 'is_signed_var': False, 'len': 10, 'name': 'VCU_ChassisThrottlePaldFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double VCUChassisThrottlePaldFb();

  // config detail: {'bit': 40, 'description': '车辆加速度', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisAccelerationFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-20|20]', 'physical_unit': 'm/s2', 'precision': 0.01, 'type': 'double'}
  double VCUChassisAccelerationFb();
};



