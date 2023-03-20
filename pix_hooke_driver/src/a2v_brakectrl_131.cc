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

#include <pix_hooke_driver/a2v_brakectrl_131.hpp>

int32_t A2vbrakectrl131::ID = 0x131;

// public
A2vbrakectrl131::A2vbrakectrl131() { Reset(); }

void A2vbrakectrl131::UpdateData(int acu_chassisbrakeen_, int acu_chassisaebctrl_, double acu_chassisbrakepdltarget_, int acu_chassisepbctrl_, int acu_brakelifesig_, int acu_checksum_131_) {
  set_p_acu_chassisbrakeen(acu_chassisbrakeen_);
  set_p_acu_chassisaebctrl(acu_chassisaebctrl_);
  set_p_acu_chassisbrakepdltarget(acu_chassisbrakepdltarget_);
  set_p_acu_chassisepbctrl(acu_chassisepbctrl_);
  set_p_acu_brakelifesig(acu_brakelifesig_);
  set_p_acu_checksum_131(acu_checksum_131_);
}

void A2vbrakectrl131::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * A2vbrakectrl131::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'description': '车辆刹车控制使能', 'enum': {0: 'ACU_CHASSISBRAKEEN_DISABLE', 1: 'ACU_CHASSISBRAKEEN_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisBrakeEn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vbrakectrl131::set_p_acu_chassisbrakeen(int acu_chassisbrakeen) {
  int x = acu_chassisbrakeen;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 4, 'description': 'AEB使能（预留，自动驾驶实现）', 'enum': {0: 'ACU_CHASSISAEBCTRL_DISABLE', 1: 'ACU_CHASSISAEBCTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisAebCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vbrakectrl131::set_p_acu_chassisaebctrl(int acu_chassisaebctrl) {
  int x = acu_chassisaebctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'description': '车辆刹车控制', 'is_signed_var': False, 'len': 10, 'name': 'ACU_ChassisBrakePdlTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void A2vbrakectrl131::set_p_acu_chassisbrakepdltarget(double acu_chassisbrakepdltarget) {
  // acu_chassisbrakepdltarget = ProtocolData::BoundedValue(0.0, 100.0, acu_chassisbrakepdltarget);
  int x = acu_chassisbrakepdltarget / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[1] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0x3;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 2);
  data[2] += to_set1.return_byte_t();
}

// config detail: {'bit': 24, 'description': '驻车控制', 'enum': {0: 'ACU_CHASSISEPBCTRL_DEFAULT', 1: 'ACU_CHASSISEPBCTRL_BRAKE', 2: 'ACU_CHASSISEPBCTRL_RELEASE'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_ChassisEpbCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vbrakectrl131::set_p_acu_chassisepbctrl(int acu_chassisepbctrl) {
  int x = acu_chassisepbctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[3] += to_set.return_byte_t();
  
}

// config detail: {'bit': 48, 'description': '循环计数0~15', 'is_signed_var': False, 'len': 4, 'name': 'ACU_BrakeLifeSig', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void A2vbrakectrl131::set_p_acu_brakelifesig(int acu_brakelifesig) {
  // acu_brakelifesig = ProtocolData::BoundedValue(0, 15, acu_brakelifesig);
  int x = acu_brakelifesig;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 4);
  data[6] += to_set.return_byte_t();
  
}

// config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'ACU_CheckSum_131', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void A2vbrakectrl131::set_p_acu_checksum_131(int acu_checksum_131) {
  // acu_checksum_131 = ProtocolData::BoundedValue(0, 255, acu_checksum_131);
  int x = acu_checksum_131;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}


