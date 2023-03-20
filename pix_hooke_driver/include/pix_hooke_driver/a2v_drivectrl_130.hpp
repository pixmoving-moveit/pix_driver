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

class A2vdrivectrl130 {
public:
	static  int32_t ID;

	A2vdrivectrl130();

  	void UpdateData(int acu_chassisdriverenctrl_, int acu_chassisdrivermodectrl_, int acu_chassisgearctrl_, double acu_chassisspeedctrl_, double acu_chassisthrottlepdltarget_, int acu_drivelifesig_, int acu_checksum_130_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'description': '车辆加速控制使能', 'enum': {0: 'ACU_CHASSISDRIVERENCTRL_DISABLE', 1: 'ACU_CHASSISDRIVERENCTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisDriverEnCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassisdriverenctrl(int acu_chassisdriverenctrl);

  // config detail: {'bit': 2, 'description': '驱动模式控制', 'enum': {0: 'ACU_CHASSISDRIVERMODECTRL_SPEED_CTRL_MODE', 1: 'ACU_CHASSISDRIVERMODECTRL_THROTTLE_CTRL_MODE', 2: 'ACU_CHASSISDRIVERMODECTRL_RESERVE', 3: 'ACU_CHASSISDRIVERMODECTRL_RESERVE'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_ChassisDriverModeCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassisdrivermodectrl(int acu_chassisdrivermodectrl);

  // config detail: {'bit': 4, 'description': '档位控制', 'enum': {0: 'ACU_CHASSISGEARCTRL_DEFAULT_N', 1: 'ACU_CHASSISGEARCTRL_D', 2: 'ACU_CHASSISGEARCTRL_N', 3: 'ACU_CHASSISGEARCTRL_R'}, 'is_signed_var': False, 'len': 4, 'name': 'ACU_ChassisGearCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassisgearctrl(int acu_chassisgearctrl);

  // config detail: {'bit': 8, 'description': '车辆速度控制', 'is_signed_var': False, 'len': 16, 'name': 'ACU_ChassisSpeedCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|50]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
  void set_p_acu_chassisspeedctrl(double acu_chassisspeedctrl);

  // config detail: {'bit': 24, 'description': '车辆油门控制', 'is_signed_var': False, 'len': 10, 'name': 'ACU_ChassisThrottlePdlTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_acu_chassisthrottlepdltarget(double acu_chassisthrottlepdltarget);

  // config detail: {'bit': 48, 'description': '循环计数0~15', 'is_signed_var': False, 'len': 4, 'name': 'ACU_DriveLifeSig', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_drivelifesig(int acu_drivelifesig);

  // config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'ACU_CheckSum_130', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_checksum_130(int acu_checksum_130);

private:
	uint8_t data[8];
};



