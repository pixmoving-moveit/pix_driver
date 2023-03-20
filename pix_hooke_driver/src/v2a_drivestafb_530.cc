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

#include <pix_hooke_driver/v2a_drivestafb_530.hpp>


V2adrivestafb530::V2adrivestafb530() {}

void V2adrivestafb530::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2adrivestafb530::Parse() {
  vcu_chassisdriverensta = VCUChassisDriverEnSta();
  vcu_chassisdiverslopover = VCUChassisDiverSlopover();
  vcu_chassisdrivermodesta = VCUChassisDriverModeSta();
  vcu_chassisgearfb = VCUChassisGearFb();
  vcu_chassisspeedfb = VCUChassisSpeedFb();
  vcu_chassisthrottlepaldfb = VCUChassisThrottlePaldFb();
  vcu_chassisaccelerationfb = VCUChassisAccelerationFb();
}


// config detail: {'bit': 0, 'description': '驱动使能状态', 'enum': {0: 'VCU_CHASSISDRIVERENSTA_DISABLE', 1: 'VCU_CHASSISDRIVERENSTA_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisDriverEnSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2adrivestafb530::VCUChassisDriverEnSta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 1, 'description': '驱动控制越界提醒', 'enum': {0: 'VCU_CHASSISDIVERSLOPOVER_NORMAL', 1: 'VCU_CHASSISDIVERSLOPOVER_OVER_SLOP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisDiverSlopover', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2adrivestafb530::VCUChassisDiverSlopover() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 2, 'description': '驱动模式反馈', 'enum': {0: 'VCU_CHASSISDRIVERMODESTA_SPEED_CTRL_MODE', 1: 'VCU_CHASSISDRIVERMODESTA_THROTTLE_CTRL_MODE', 2: 'VCU_CHASSISDRIVERMODESTA_RESERVE', 3: 'VCU_CHASSISDRIVERMODESTA_RESERVE'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisDriverModeSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2adrivestafb530::VCUChassisDriverModeSta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 4, 'description': '档位状态', 'enum': {0: 'VCU_CHASSISGEARFB_NO_USE', 1: 'VCU_CHASSISGEARFB_D', 2: 'VCU_CHASSISGEARFB_N', 3: 'VCU_CHASSISGEARFB_R'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisGearFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2adrivestafb530::VCUChassisGearFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '车速实际反馈', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisSpeedFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-50|50]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
double V2adrivestafb530::VCUChassisSpeedFb() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 24, 'description': '油门请求值反馈', 'is_signed_var': False, 'len': 10, 'name': 'VCU_ChassisThrottlePaldFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double V2adrivestafb530::VCUChassisThrottlePaldFb() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 2);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 40, 'description': '车辆加速度', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisAccelerationFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-20|20]', 'physical_unit': 'm/s2', 'precision': 0.01, 'type': 'double'}
double V2adrivestafb530::VCUChassisAccelerationFb() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

