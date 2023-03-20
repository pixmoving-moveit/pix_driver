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

#include <pix_hooke_driver/v2a_steerstafb_532.hpp>


V2asteerstafb532::V2asteerstafb532() {}

void V2asteerstafb532::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2asteerstafb532::Parse() {
  vcu_chassissteerensta = VCUChassisSteerEnSta();
  vcu_chassissteerslopover = VCUChassisSteerSlopover();
  vcu_chassissteerworkmode = VCUChassisSteerWorkMode();
  vcu_chassissteermodefb = VCUChassisSteerModeFb();
  vcu_chassissteeranglefb = VCUChassisSteerAngleFb();
  vcu_chassissteeranglerearfb = VCUChassisSteerAngleRearFb();
  vcu_chassissteeranglespeedfb = VCUChassisSteerAngleSpeedFb();
}


// config detail: {'bit': 0, 'description': '转向使能状态', 'enum': {0: 'VCU_CHASSISSTEERENSTA_DISABLE', 1: 'VCU_CHASSISSTEERENSTA_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisSteerEnSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2asteerstafb532::VCUChassisSteerEnSta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 1, 'description': '转向控制越界提醒', 'enum': {0: 'VCU_CHASSISSTEERSLOPOVER_NORMAL', 1: 'VCU_CHASSISSTEERSLOPOVER_OVER_SLOP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisSteerSlopover', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2asteerstafb532::VCUChassisSteerSlopover() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 2, 'description': '转向线控模式反馈', 'enum': {0: 'VCU_CHASSISSTEERWORKMODE_MACHINE', 1: 'VCU_CHASSISSTEERWORKMODE_WIRE', 2: 'VCU_CHASSISSTEERWORKMODE_POWER'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisSteerWorkMode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2asteerstafb532::VCUChassisSteerWorkMode() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 4, 'description': '转向模式反馈', 'enum': {0: 'VCU_CHASSISSTEERMODEFB_FRONT_ACKERMAN', 1: 'VCU_CHASSISSTEERMODEFB_SAME_FRONT_AND_BACK', 2: 'VCU_CHASSISSTEERMODEFB_FRONT_DIFFERENT_BACK', 3: 'VCU_CHASSISSTEERMODEFB_BACK_ACKRMAN', 4: 'VCU_CHASSISSTEERMODEFB_FRONT_BACK'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_ChassisSteerModeFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2asteerstafb532::VCUChassisSteerModeFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 4);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '前转向方向盘转角反馈', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisSteerAngleFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
int V2asteerstafb532::VCUChassisSteerAngleFb() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'description': '后转向方向盘转角反馈', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisSteerAngleRearFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
int V2asteerstafb532::VCUChassisSteerAngleRearFb() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'description': '设置的转向转角速度反馈', 'is_signed_var': False, 'len': 8, 'name': 'VCU_ChassisSteerAngleSpeedFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|500]', 'physical_unit': 'deg/s', 'precision': 2.0, 'type': 'double'}
double V2asteerstafb532::VCUChassisSteerAngleSpeedFb() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

