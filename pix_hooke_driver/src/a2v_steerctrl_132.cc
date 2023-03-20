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

#include <pix_hooke_driver/a2v_steerctrl_132.hpp>

int32_t A2vsteerctrl132::ID = 0x132;

// public
A2vsteerctrl132::A2vsteerctrl132() { Reset(); }

void A2vsteerctrl132::UpdateData(int acu_chassissteerenctrl_, int acu_chassissteermodectrl_, int acu_chassissteerangletarget_, int acu_chassissteeranglereartarget_, double acu_chassissteeranglespeedctrl_, int acu_checksum_132_) {
  set_p_acu_chassissteerenctrl(acu_chassissteerenctrl_);
  set_p_acu_chassissteermodectrl(acu_chassissteermodectrl_);
  set_p_acu_chassissteerangletarget(acu_chassissteerangletarget_);
  set_p_acu_chassissteeranglereartarget(acu_chassissteeranglereartarget_);
  set_p_acu_chassissteeranglespeedctrl(acu_chassissteeranglespeedctrl_);
  set_p_acu_checksum_132(acu_checksum_132_);
}

void A2vsteerctrl132::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * A2vsteerctrl132::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'description': '车辆转向控制使能', 'enum': {0: 'ACU_CHASSISSTEERENCTRL_DISABLE', 1: 'ACU_CHASSISSTEERENCTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisSteerEnCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vsteerctrl132::set_p_acu_chassissteerenctrl(int acu_chassissteerenctrl) {
  int x = acu_chassissteerenctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 4, 'description': '转向模式控制', 'enum': {0: 'ACU_CHASSISSTEERMODECTRL_FRONT_ACKERMAN', 1: 'ACU_CHASSISSTEERMODECTRL_SAME_FRONT_AND_BACK', 2: 'ACU_CHASSISSTEERMODECTRL_FRONT_DIFFERENT_BACK', 3: 'ACU_CHASSISSTEERMODECTRL_BACK_ACKRMAN', 4: 'ACU_CHASSISSTEERMODECTRL_FRONT_BACK'}, 'is_signed_var': False, 'len': 4, 'name': 'ACU_ChassisSteerModeCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vsteerctrl132::set_p_acu_chassissteermodectrl(int acu_chassissteermodectrl) {
  int x = acu_chassissteermodectrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 4);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'description': '车辆转向控制（前）', 'is_signed_var': True, 'len': 16, 'name': 'ACU_ChassisSteerAngleTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
void A2vsteerctrl132::set_p_acu_chassissteerangletarget(int acu_chassissteerangletarget) {
  // acu_chassissteerangletarget = ProtocolData::BoundedValue(-500, 500, acu_chassissteerangletarget);
  int x = acu_chassissteerangletarget;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[1] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[2] += to_set1.return_byte_t();
}

// config detail: {'bit': 24, 'description': '车辆转向控制（后）', 'is_signed_var': True, 'len': 16, 'name': 'ACU_ChassisSteerAngleRearTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
void A2vsteerctrl132::set_p_acu_chassissteeranglereartarget(int acu_chassissteeranglereartarget) {
  // acu_chassissteeranglereartarget = ProtocolData::BoundedValue(-500, 500, acu_chassissteeranglereartarget);
  int x = acu_chassissteeranglereartarget;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[3] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[4] += to_set1.return_byte_t();
}

// config detail: {'bit': 40, 'description': '车辆方向盘转角速度控制', 'is_signed_var': False, 'len': 8, 'name': 'ACU_ChassisSteerAngleSpeedCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|500]', 'physical_unit': 'deg/s', 'precision': 2.0, 'type': 'double'}
void A2vsteerctrl132::set_p_acu_chassissteeranglespeedctrl(double acu_chassissteeranglespeedctrl) {
  // acu_chassissteeranglespeedctrl = ProtocolData::BoundedValue(0.0, 500.0, acu_chassissteeranglespeedctrl);
  int x = acu_chassissteeranglespeedctrl / 2.000000;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[5] += to_set.return_byte_t();
  
}

// config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'ACU_CheckSum_132', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void A2vsteerctrl132::set_p_acu_checksum_132(int acu_checksum_132) {
  // acu_checksum_132 = ProtocolData::BoundedValue(0, 255, acu_checksum_132);
  int x = acu_checksum_132;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}


