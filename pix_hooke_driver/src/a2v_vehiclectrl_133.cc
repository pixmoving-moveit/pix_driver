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

#include <pix_hooke_driver/a2v_vehiclectrl_133.hpp>

int32_t A2vvehiclectrl133::ID = 0x133;

// public
A2vvehiclectrl133::A2vvehiclectrl133() { Reset(); }

void A2vvehiclectrl133::UpdateData(int acu_vehicleposlampctrl_, int acu_vehicleheadlampctrl_, int acu_vehicleleftlampctrl_, int acu_vehiclerightlampctrl_, int acu_vehiclehighbeamctrl_, int acu_vehiclefoglampctrl_, int acu_vehiclebodylightctrl_, int acu_vehiclereadlightctrl_, int acu_vehiclevoice_, int acu_vehiclewipersctrl_, int acu_vehicledoorctrl_, int acu_vehiclewindowctrl_, int acu_chassisspeedlimitemode_, int acu_chassisspeedlimiteval_, int acu_checksumen_) {
  set_p_acu_vehicleposlampctrl(acu_vehicleposlampctrl_);
  set_p_acu_vehicleheadlampctrl(acu_vehicleheadlampctrl_);
  set_p_acu_vehicleleftlampctrl(acu_vehicleleftlampctrl_);
  set_p_acu_vehiclerightlampctrl(acu_vehiclerightlampctrl_);
  set_p_acu_vehiclehighbeamctrl(acu_vehiclehighbeamctrl_);
  set_p_acu_vehiclefoglampctrl(acu_vehiclefoglampctrl_);
  set_p_acu_vehiclebodylightctrl(acu_vehiclebodylightctrl_);
  set_p_acu_vehiclereadlightctrl(acu_vehiclereadlightctrl_);
  set_p_acu_vehiclevoice(acu_vehiclevoice_);
  set_p_acu_vehiclewipersctrl(acu_vehiclewipersctrl_);
  set_p_acu_vehicledoorctrl(acu_vehicledoorctrl_);
  set_p_acu_vehiclewindowctrl(acu_vehiclewindowctrl_);
  set_p_acu_chassisspeedlimitemode(acu_chassisspeedlimitemode_);
  set_p_acu_chassisspeedlimiteval(acu_chassisspeedlimiteval_);
  set_p_acu_checksumen(acu_checksumen_);
}

void A2vvehiclectrl133::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * A2vvehiclectrl133::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'description': '位置灯控制', 'enum': {0: 'ACU_VEHICLEPOSLAMPCTRL_OFF', 1: 'ACU_VEHICLEPOSLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehiclePosLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehicleposlampctrl(int acu_vehicleposlampctrl) {
  int x = acu_vehicleposlampctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 1, 'description': '近光灯控制', 'enum': {0: 'ACU_VEHICLEHEADLAMPCTRL_OFF', 1: 'ACU_VEHICLEHEADLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleHeadLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehicleheadlampctrl(int acu_vehicleheadlampctrl) {
  int x = acu_vehicleheadlampctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 1, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 2, 'description': '左转向灯控制', 'enum': {0: 'ACU_VEHICLELEFTLAMPCTRL_OFF', 1: 'ACU_VEHICLELEFTLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleLeftLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehicleleftlampctrl(int acu_vehicleleftlampctrl) {
  int x = acu_vehicleleftlampctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 3, 'description': '右转向灯控制', 'enum': {0: 'ACU_VEHICLERIGHTLAMPCTRL_OFF', 1: 'ACU_VEHICLERIGHTLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleRightLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclerightlampctrl(int acu_vehiclerightlampctrl) {
  int x = acu_vehiclerightlampctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 3, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 4, 'description': '远光灯控制', 'enum': {0: 'ACU_VEHICLEHIGHBEAMCTRL_OFF', 1: 'ACU_VEHICLEHIGHBEAMCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleHighBeamCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclehighbeamctrl(int acu_vehiclehighbeamctrl) {
  int x = acu_vehiclehighbeamctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 5, 'description': '雾灯控制', 'enum': {0: 'ACU_VEHICLEFOGLAMPCTRL_OFF', 1: 'ACU_VEHICLEFOGLAMPCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleFogLampCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclefoglampctrl(int acu_vehiclefoglampctrl) {
  int x = acu_vehiclefoglampctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 5, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 6, 'description': '氛围灯控制', 'enum': {0: 'ACU_VEHICLEBODYLIGHTCTRL_OFF', 1: 'ACU_VEHICLEBODYLIGHTCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleBodyLightCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclebodylightctrl(int acu_vehiclebodylightctrl) {
  int x = acu_vehiclebodylightctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 6, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 7, 'description': '车内灯光总控制', 'enum': {0: 'ACU_VEHICLEREADLIGHTCTRL_OFF', 1: 'ACU_VEHICLEREADLIGHTCTRL_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_VehicleReadLightCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclereadlightctrl(int acu_vehiclereadlightctrl) {
  int x = acu_vehiclereadlightctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 7, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'description': '提示音控制', 'enum': {0: 'ACU_VEHICLEVOICE_OFF', 1: 'ACU_VEHICLEVOICE_TURN_LEFT', 2: 'ACU_VEHICLEVOICE_TURN_RIGHT', 3: 'ACU_VEHICLEVOICE_BACKWARD'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_VehicleVoice', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclevoice(int acu_vehiclevoice) {
  int x = acu_vehiclevoice;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 10, 'description': '雨刮控制', 'enum': {0: 'ACU_VEHICLEWIPERSCTRL_OFF', 1: 'ACU_VEHICLEWIPERSCTRL_LOW', 2: 'ACU_VEHICLEWIPERSCTRL_MID', 3: 'ACU_VEHICLEWIPERSCTRL_HIGH'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_VehicleWipersCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclewipersctrl(int acu_vehiclewipersctrl) {
  int x = acu_vehiclewipersctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 12, 'description': '车门控制', 'enum': {0: 'ACU_VEHICLEDOORCTRL_DEFAULT', 1: 'ACU_VEHICLEDOORCTRL_CLOSE', 2: 'ACU_VEHICLEDOORCTRL_OPEN'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_VehicleDoorCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehicledoorctrl(int acu_vehicledoorctrl) {
  int x = acu_vehicledoorctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 14, 'description': '车窗控制（预留）', 'enum': {0: 'ACU_VEHICLEWINDOWCTRL_DEFAULT', 1: 'ACU_VEHICLEWINDOWCTRL_CLOSE', 2: 'ACU_VEHICLEWINDOWCTRL_OPEN'}, 'is_signed_var': False, 'len': 3, 'name': 'ACU_VehicleWindowCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_vehiclewindowctrl(int acu_vehiclewindowctrl) {
  int x = acu_vehiclewindowctrl;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(a);
  to_set0.set_value(t, 6, 2);
  data[1] += to_set0.return_byte_t();
  x >>= 2;

  t = x & 0x1;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 1);
  data[2] += to_set1.return_byte_t();
}

// config detail: {'bit': 24, 'description': '限速控制', 'enum': {0: 'ACU_CHASSISSPEEDLIMITEMODE_DEFAULT', 1: 'ACU_CHASSISSPEEDLIMITEMODE_LIMIT'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisSpeedLimiteMode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_chassisspeedlimitemode(int acu_chassisspeedlimitemode) {
  int x = acu_chassisspeedlimitemode;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[3] += to_set.return_byte_t();
  
}

// config detail: {'bit': 32, 'description': '速度限制值', 'is_signed_var': False, 'len': 16, 'name': 'ACU_ChassisSpeedLimiteVal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[5|20]', 'physical_unit': 'm/s', 'precision': 1.0, 'type': 'int'}
void A2vvehiclectrl133::set_p_acu_chassisspeedlimiteval(int acu_chassisspeedlimiteval) {
  // acu_chassisspeedlimiteval = ProtocolData::BoundedValue(5, 20, acu_chassisspeedlimiteval);
  int x = acu_chassisspeedlimiteval;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[4] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[5] += to_set1.return_byte_t();
}

// config detail: {'bit': 48, 'description': '校验模式使能(预留)', 'enum': {0: 'ACU_CHECKSUMEN_ENABLE', 1: 'ACU_CHECKSUMEN_DISABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_CheckSumEn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vvehiclectrl133::set_p_acu_checksumen(int acu_checksumen) {
  int x = acu_checksumen;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[6] += to_set.return_byte_t();
  
}


