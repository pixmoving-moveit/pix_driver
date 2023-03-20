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

class V2achassiswheelanglefb541 {
public:
    static const uint32_t ID = 0x541;
    V2achassiswheelanglefb541();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vcu_chassiswheelanglelf;
    double vcu_chassiswheelanglerf;
    double vcu_chassiswheelanglelr;
    double vcu_chassiswheelanglerr;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '左前轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleLf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
  double VCUChassisWheelAngleLf();

  // config detail: {'bit': 16, 'description': '右前轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleRf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
  double VCUChassisWheelAngleRf();

  // config detail: {'bit': 32, 'description': '左后轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleLr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
  double VCUChassisWheelAngleLr();

  // config detail: {'bit': 48, 'description': '右后轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleRr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
  double VCUChassisWheelAngleRr();
};



