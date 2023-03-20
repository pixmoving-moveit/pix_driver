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

class A2vsteerctrl132 {
public:
	static  int32_t ID;

	A2vsteerctrl132();

  	void UpdateData(int acu_chassissteerenctrl_, int acu_chassissteermodectrl_, int acu_chassissteerangletarget_, int acu_chassissteeranglereartarget_, double acu_chassissteeranglespeedctrl_, int acu_checksum_132_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'description': '车辆转向控制使能', 'enum': {0: 'ACU_CHASSISSTEERENCTRL_DISABLE', 1: 'ACU_CHASSISSTEERENCTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisSteerEnCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassissteerenctrl(int acu_chassissteerenctrl);

  // config detail: {'bit': 4, 'description': '转向模式控制', 'enum': {0: 'ACU_CHASSISSTEERMODECTRL_FRONT_ACKERMAN', 1: 'ACU_CHASSISSTEERMODECTRL_SAME_FRONT_AND_BACK', 2: 'ACU_CHASSISSTEERMODECTRL_FRONT_DIFFERENT_BACK', 3: 'ACU_CHASSISSTEERMODECTRL_BACK_ACKRMAN', 4: 'ACU_CHASSISSTEERMODECTRL_FRONT_BACK'}, 'is_signed_var': False, 'len': 4, 'name': 'ACU_ChassisSteerModeCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassissteermodectrl(int acu_chassissteermodectrl);

  // config detail: {'bit': 8, 'description': '车辆转向控制（前）', 'is_signed_var': True, 'len': 16, 'name': 'ACU_ChassisSteerAngleTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_chassissteerangletarget(int acu_chassissteerangletarget);

  // config detail: {'bit': 24, 'description': '车辆转向控制（后）', 'is_signed_var': True, 'len': 16, 'name': 'ACU_ChassisSteerAngleRearTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_chassissteeranglereartarget(int acu_chassissteeranglereartarget);

  // config detail: {'bit': 40, 'description': '车辆方向盘转角速度控制', 'is_signed_var': False, 'len': 8, 'name': 'ACU_ChassisSteerAngleSpeedCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|500]', 'physical_unit': 'deg/s', 'precision': 2.0, 'type': 'double'}
  void set_p_acu_chassissteeranglespeedctrl(double acu_chassissteeranglespeedctrl);

  // config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'ACU_CheckSum_132', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_checksum_132(int acu_checksum_132);

private:
	uint8_t data[8];
};



