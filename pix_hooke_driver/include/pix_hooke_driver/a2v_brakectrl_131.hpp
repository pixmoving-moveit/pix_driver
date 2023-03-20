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

class A2vbrakectrl131 {
public:
	static  int32_t ID;

	A2vbrakectrl131();

  	void UpdateData(int acu_chassisbrakeen_, int acu_chassisaebctrl_, double acu_chassisbrakepdltarget_, int acu_chassisepbctrl_, int acu_brakelifesig_, int acu_checksum_131_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'description': '车辆刹车控制使能', 'enum': {0: 'ACU_CHASSISBRAKEEN_DISABLE', 1: 'ACU_CHASSISBRAKEEN_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisBrakeEn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassisbrakeen(int acu_chassisbrakeen);

  // config detail: {'bit': 4, 'description': 'AEB使能（预留，自动驾驶实现）', 'enum': {0: 'ACU_CHASSISAEBCTRL_DISABLE', 1: 'ACU_CHASSISAEBCTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisAebCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassisaebctrl(int acu_chassisaebctrl);

  // config detail: {'bit': 8, 'description': '车辆刹车控制', 'is_signed_var': False, 'len': 10, 'name': 'ACU_ChassisBrakePdlTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_acu_chassisbrakepdltarget(double acu_chassisbrakepdltarget);

  // config detail: {'bit': 24, 'description': '驻车控制', 'enum': {0: 'ACU_CHASSISEPBCTRL_DEFAULT', 1: 'ACU_CHASSISEPBCTRL_BRAKE', 2: 'ACU_CHASSISEPBCTRL_RELEASE'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_ChassisEpbCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassisepbctrl(int acu_chassisepbctrl);

  // config detail: {'bit': 48, 'description': '循环计数0~15', 'is_signed_var': False, 'len': 4, 'name': 'ACU_BrakeLifeSig', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_brakelifesig(int acu_brakelifesig);

  // config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'ACU_CheckSum_131', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_checksum_131(int acu_checksum_131);

private:
	uint8_t data[8];
};



