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

#include <pix_hooke_driver/v2a_chassiswheelanglefb_541.hpp>


V2achassiswheelanglefb541::V2achassiswheelanglefb541() {}

void V2achassiswheelanglefb541::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2achassiswheelanglefb541::Parse() {
  vcu_chassiswheelanglelf = VCUChassisWheelAngleLf();
  vcu_chassiswheelanglerf = VCUChassisWheelAngleRf();
  vcu_chassiswheelanglelr = VCUChassisWheelAngleLr();
  vcu_chassiswheelanglerr = VCUChassisWheelAngleRr();
}


// config detail: {'bit': 0, 'description': '左前轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleLf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
double V2achassiswheelanglefb541::VCUChassisWheelAngleLf() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 16, 'description': '右前轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleRf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
double V2achassiswheelanglefb541::VCUChassisWheelAngleRf() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 32, 'description': '左后轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleLr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
double V2achassiswheelanglefb541::VCUChassisWheelAngleLr() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 48, 'description': '右后轮转向转角度', 'is_signed_var': True, 'len': 12, 'name': 'VCU_ChassisWheelAngleRr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-90|90]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
double V2achassiswheelanglefb541::VCUChassisWheelAngleRr() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

