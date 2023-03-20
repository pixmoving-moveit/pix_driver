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

class V2achassiswheeltirepressfb540 {
public:
    static const uint32_t ID = 0x540;
    V2achassiswheeltirepressfb540();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vcu_chassiswheeltirepresslf;
    double vcu_chassiswheeltirepressrf;
    double vcu_chassiswheeltirepresslr;
    double vcu_chassiswheeltirepressrr;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '左前轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressLf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
  double VCUChassisWheelTirePressLf();

  // config detail: {'bit': 16, 'description': '右前轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressRf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
  double VCUChassisWheelTirePressRf();

  // config detail: {'bit': 32, 'description': '左后轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressLr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
  double VCUChassisWheelTirePressLr();

  // config detail: {'bit': 48, 'description': '右后轮胎压', 'is_signed_var': False, 'len': 12, 'name': 'VCU_ChassisWheelTirePressRr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|20]', 'physical_unit': 'bar', 'precision': 0.01, 'type': 'double'}
  double VCUChassisWheelTirePressRr();
};



