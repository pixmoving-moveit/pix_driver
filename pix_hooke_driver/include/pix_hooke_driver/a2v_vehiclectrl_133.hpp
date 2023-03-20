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

class A2vvehiclectrl133 {
public:
	static  int32_t ID;

	A2vvehiclectrl133();

  	void UpdateData(int acu_vehicleposlampctrl_, int acu_vehicleheadlampctrl_, int acu_vehicleleftlampctrl_, int acu_vehiclerightlampctrl_, int acu_vehiclehighbeamctrl_, int acu_vehiclefoglampctrl_, int acu_vehiclebodylightctrl_, int acu_vehiclereadlightctrl_, int acu_vehiclevoice_, int acu_vehiclewipersctrl_, int acu_vehicledoorctrl_, int acu_vehiclewindowctrl_, int acu_chassisspeedlimitemode_, int acu_chassisspeedlimiteval_, int acu_checksumen_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'description': '位置灯控制', 'enum': {0: 'ACU_VEHICLEPOSLAMPCTRL_OFF', 1: 'ACU_VEHICLEPOSLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehiclePosLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehicleposlampctrl(int acu_vehicleposlampctrl);

  // config detail: {'bit': 1, 'description': '近光灯控制', 'enum': {0: 'ACU_VEHICLEHEADLAMPCTRL_OFF', 1: 'ACU_VEHICLEHEADLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleHeadLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehicleheadlampctrl(int acu_vehicleheadlampctrl);

  // config detail: {'bit': 2, 'description': '左转向灯控制', 'enum': {0: 'ACU_VEHICLELEFTLAMPCTRL_OFF', 1: 'ACU_VEHICLELEFTLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleLeftLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehicleleftlampctrl(int acu_vehicleleftlampctrl);

  // config detail: {'bit': 3, 'description': '右转向灯控制', 'enum': {0: 'ACU_VEHICLERIGHTLAMPCTRL_OFF', 1: 'ACU_VEHICLERIGHTLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleRightLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclerightlampctrl(int acu_vehiclerightlampctrl);

  // config detail: {'bit': 4, 'description': '远光灯控制', 'enum': {0: 'ACU_VEHICLEHIGHBEAMCTRL_OFF', 1: 'ACU_VEHICLEHIGHBEAMCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleHighBeamCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclehighbeamctrl(int acu_vehiclehighbeamctrl);

  // config detail: {'bit': 5, 'description': '雾灯控制', 'enum': {0: 'ACU_VEHICLEFOGLAMPCTRL_OFF', 1: 'ACU_VEHICLEFOGLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleFogLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclefoglampctrl(int acu_vehiclefoglampctrl);

  // config detail: {'bit': 6, 'description': '氛围灯控制', 'enum': {0: 'ACU_VEHICLEBODYLIGHTCTRL_OFF', 1: 'ACU_VEHICLEBODYLIGHTCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleBodyLightCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclebodylightctrl(int acu_vehiclebodylightctrl);

  // config detail: {'bit': 7, 'description': '车内灯光总控制', 'enum': {0: 'ACU_VEHICLEREADLIGHTCTRL_OFF', 1: 'ACU_VEHICLEREADLIGHTCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleReadLightCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclereadlightctrl(int acu_vehiclereadlightctrl);

  // config detail: {'bit': 8, 'description': '提示音控制', 'enum': {0: 'ACU_VEHICLEVOICE_OFF', 1: 'ACU_VEHICLEVOICE_TURN_LEFT', 2: 'ACU_VEHICLEVOICE_TURN_RIGHT', 3: 'ACU_VEHICLEVOICE_BACKWARD'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_VehicleVoice', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclevoice(int acu_vehiclevoice);

  // config detail: {'bit': 10, 'description': '雨刮控制', 'enum': {0: 'ACU_VEHICLEWIPERSCTRL_OFF', 1: 'ACU_VEHICLEWIPERSCTRL_LOW', 2: 'ACU_VEHICLEWIPERSCTRL_MID', 3: 'ACU_VEHICLEWIPERSCTRL_HIGH'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_VehicleWipersCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclewipersctrl(int acu_vehiclewipersctrl);

  // config detail: {'bit': 12, 'description': '车门控制', 'enum': {0: 'ACU_VEHICLEDOORCTRL_DEFAULT', 1: 'ACU_VEHICLEDOORCTRL_CLOSE', 2: 'ACU_VEHICLEDOORCTRL_OPEN'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_VehicleDoorCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehicledoorctrl(int acu_vehicledoorctrl);

  // config detail: {'bit': 14, 'description': '车窗控制（预留）', 'enum': {0: 'ACU_VEHICLEWINDOWCTRL_DEFAULT', 1: 'ACU_VEHICLEWINDOWCTRL_CLOSE', 2: 'ACU_VEHICLEWINDOWCTRL_OPEN'}, 'is_signed_var': False, 'len': 3, 'name': 'ACU_VehicleWindowCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_vehiclewindowctrl(int acu_vehiclewindowctrl);

  // config detail: {'bit': 24, 'description': '限速控制', 'enum': {0: 'ACU_CHASSISSPEEDLIMITEMODE_DEFAULT', 1: 'ACU_CHASSISSPEEDLIMITEMODE_LIMIT'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisSpeedLimiteMode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_chassisspeedlimitemode(int acu_chassisspeedlimitemode);

  // config detail: {'bit': 32, 'description': '速度限制值', 'is_signed_var': False, 'len': 16, 'name': 'ACU_ChassisSpeedLimiteVal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[5|20]', 'physical_unit': 'm/s', 'precision': 1.0, 'type': 'int'}
  void set_p_acu_chassisspeedlimiteval(int acu_chassisspeedlimiteval);

  // config detail: {'bit': 48, 'description': '校验模式使能(预留)', 'enum': {0: 'ACU_CHECKSUMEN_ENABLE', 1: 'ACU_CHECKSUMEN_DISABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_CheckSumEn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_acu_checksumen(int acu_checksumen);

private:
	uint8_t data[8];
};



