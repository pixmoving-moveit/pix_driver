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

#include <pix_hooke_driver/a2v_drivectrl_130.hpp>

int32_t A2vdrivectrl130::ID = 0x130;

// public
A2vdrivectrl130::A2vdrivectrl130() { Reset(); }

void A2vdrivectrl130::UpdateData(int acu_chassisdriverenctrl_, int acu_chassisdrivermodectrl_, int acu_chassisgearctrl_, double acu_chassisspeedctrl_, double acu_chassisthrottlepdltarget_, int acu_drivelifesig_, int acu_checksum_130_) {
  set_p_acu_chassisdriverenctrl(acu_chassisdriverenctrl_);
  set_p_acu_chassisdrivermodectrl(acu_chassisdrivermodectrl_);
  set_p_acu_chassisgearctrl(acu_chassisgearctrl_);
  set_p_acu_chassisspeedctrl(acu_chassisspeedctrl_);
  set_p_acu_chassisthrottlepdltarget(acu_chassisthrottlepdltarget_);
  set_p_acu_drivelifesig(acu_drivelifesig_);
  set_p_acu_checksum_130(acu_checksum_130_);
}

void A2vdrivectrl130::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * A2vdrivectrl130::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'description': '车辆加速控制使能', 'enum': {0: 'ACU_CHASSISDRIVERENCTRL_DISABLE', 1: 'ACU_CHASSISDRIVERENCTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'ACU_ChassisDriverEnCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vdrivectrl130::set_p_acu_chassisdriverenctrl(int acu_chassisdriverenctrl) {
  int x = acu_chassisdriverenctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 2, 'description': '驱动模式控制', 'enum': {0: 'ACU_CHASSISDRIVERMODECTRL_SPEED_CTRL_MODE', 1: 'ACU_CHASSISDRIVERMODECTRL_THROTTLE_CTRL_MODE', 2: 'ACU_CHASSISDRIVERMODECTRL_RESERVE', 3: 'ACU_CHASSISDRIVERMODECTRL_RESERVE'}, 'is_signed_var': False, 'len': 2, 'name': 'ACU_ChassisDriverModeCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vdrivectrl130::set_p_acu_chassisdrivermodectrl(int acu_chassisdrivermodectrl) {
  int x = acu_chassisdrivermodectrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 2);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 4, 'description': '档位控制', 'enum': {0: 'ACU_CHASSISGEARCTRL_DEFAULT_N', 1: 'ACU_CHASSISGEARCTRL_D', 2: 'ACU_CHASSISGEARCTRL_N', 3: 'ACU_CHASSISGEARCTRL_R'}, 'is_signed_var': False, 'len': 4, 'name': 'ACU_ChassisGearCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void A2vdrivectrl130::set_p_acu_chassisgearctrl(int acu_chassisgearctrl) {
  int x = acu_chassisgearctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 4);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'description': '车辆速度控制', 'is_signed_var': False, 'len': 16, 'name': 'ACU_ChassisSpeedCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|50]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
void A2vdrivectrl130::set_p_acu_chassisspeedctrl(double acu_chassisspeedctrl) {
  // acu_chassisspeedctrl = ProtocolData::BoundedValue(0.0, 50.0, acu_chassisspeedctrl);
  int x = acu_chassisspeedctrl / 0.010000;
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

// config detail: {'bit': 24, 'description': '车辆油门控制', 'is_signed_var': False, 'len': 10, 'name': 'ACU_ChassisThrottlePdlTarget', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void A2vdrivectrl130::set_p_acu_chassisthrottlepdltarget(double acu_chassisthrottlepdltarget) {
  // acu_chassisthrottlepdltarget = ProtocolData::BoundedValue(0.0, 100.0, acu_chassisthrottlepdltarget);
  int x = acu_chassisthrottlepdltarget / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[3] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0x3;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 2);
  data[4] += to_set1.return_byte_t();
}

// config detail: {'bit': 48, 'description': '循环计数0~15', 'is_signed_var': False, 'len': 4, 'name': 'ACU_DriveLifeSig', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void A2vdrivectrl130::set_p_acu_drivelifesig(int acu_drivelifesig) {
  // acu_drivelifesig = ProtocolData::BoundedValue(0, 15, acu_drivelifesig);
  int x = acu_drivelifesig;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 4);
  data[6] += to_set.return_byte_t();
  
}

// config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'ACU_CheckSum_130', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void A2vdrivectrl130::set_p_acu_checksum_130(int acu_checksum_130) {
  // acu_checksum_130 = ProtocolData::BoundedValue(0, 255, acu_checksum_130);
  int x = acu_checksum_130;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}


