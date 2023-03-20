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

#include <pix_hooke_driver/v2a_brakestafb_531.hpp>


V2abrakestafb531::V2abrakestafb531() {}

void V2abrakestafb531::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2abrakestafb531::Parse() {
  vcu_chassisbrakeensta = VCUChassisBrakeEnSta();
  vcu_vehiclebrakelampfb = VCUVehicleBrakeLampFb();
  vcu_chassisepbfb = VCUChassisEpbFb();
  vcu_chassisbrakepadlfb = VCUChassisBrakePadlFb();
  vcu_aebenstafb = VCUAebEnStaFb();
  vcu_aebtriggerstafb = VCUAebTriggerStaFb();
}


// config detail: {'bit': 0, 'description': '制动使能状态', 'enum': {0: 'VCU_CHASSISBRAKEENSTA_DISABLE', 1: 'VCU_CHASSISBRAKEENSTA_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisBrakeEnSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2abrakestafb531::VCUChassisBrakeEnSta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 2, 'description': '制动灯状态反馈', 'enum': {0: 'VCU_VEHICLEBRAKELAMPFB_OFF', 1: 'VCU_VEHICLEBRAKELAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleBrakeLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2abrakestafb531::VCUVehicleBrakeLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 4, 'description': '驻车状态', 'enum': {0: 'VCU_CHASSISEPBFB_RELEASE', 1: 'VCU_CHASSISEPBFB_BRAKE', 2: 'VCU_CHASSISEPBFB_RELEASING', 3: 'VCU_CHASSISEPBFB_BRAKING'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisEpbFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2abrakestafb531::VCUChassisEpbFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '制动实际反馈', 'is_signed_var': False, 'len': 10, 'name': 'VCU_ChassisBrakePadlFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double V2abrakestafb531::VCUChassisBrakePadlFb() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 2);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 24, 'description': 'AEB开启状态反馈', 'enum': {0: 'VCU_AEBENSTAFB_OFF', 1: 'VCU_AEBENSTAFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_AebEnStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2abrakestafb531::VCUAebEnStaFb() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 26, 'description': 'AEB触发状态反馈', 'enum': {0: 'VCU_AEBTRIGGERSTAFB_OFF', 1: 'VCU_AEBTRIGGERSTAFB_AEB_TRIGGER'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_AebTriggerStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2abrakestafb531::VCUAebTriggerStaFb() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(2, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

