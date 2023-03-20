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
#include <iostream>

class V2abrakestafb531 {
public:
    static const uint32_t ID = 0x531;
    V2abrakestafb531();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int vcu_chassisbrakeensta;
    int vcu_vehiclebrakelampfb;
    int vcu_chassisepbfb;
    double vcu_chassisbrakepadlfb;
    int vcu_aebenstafb;
    int vcu_aebtriggerstafb;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '制动使能状态', 'enum': {0: 'VCU_CHASSISBRAKEENSTA_DISABLE', 1: 'VCU_CHASSISBRAKEENSTA_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisBrakeEnSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisBrakeEnSta();

  // config detail: {'bit': 2, 'description': '制动灯状态反馈', 'enum': {0: 'VCU_VEHICLEBRAKELAMPFB_OFF', 1: 'VCU_VEHICLEBRAKELAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleBrakeLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleBrakeLampFb();

  // config detail: {'bit': 4, 'description': '驻车状态', 'enum': {0: 'VCU_CHASSISEPBFB_RELEASE', 1: 'VCU_CHASSISEPBFB_BRAKE', 2: 'VCU_CHASSISEPBFB_RELEASING', 3: 'VCU_CHASSISEPBFB_BRAKING'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisEpbFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisEpbFb();

  // config detail: {'bit': 8, 'description': '制动实际反馈', 'is_signed_var': False, 'len': 10, 'name': 'VCU_ChassisBrakePadlFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double VCUChassisBrakePadlFb();

  // config detail: {'bit': 24, 'description': 'AEB开启状态反馈', 'enum': {0: 'VCU_AEBENSTAFB_OFF', 1: 'VCU_AEBENSTAFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_AebEnStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUAebEnStaFb();

  // config detail: {'bit': 26, 'description': 'AEB触发状态反馈', 'enum': {0: 'VCU_AEBTRIGGERSTAFB_OFF', 1: 'VCU_AEBTRIGGERSTAFB_AEB_TRIGGER'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_AebTriggerStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUAebTriggerStaFb();
};



