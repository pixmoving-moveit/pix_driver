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

#include <pix_hooke_driver/v2a_powerstafb_535.hpp>


V2apowerstafb535::V2apowerstafb535() {}

void V2apowerstafb535::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2apowerstafb535::Parse() {
  vcu_chassisbmsreserved_1 = VCUChassisBmsReserved1();
  vcu_chassispowerchargesta = VCUChassisPowerChargeSta();
  vcu_chassispowerchargesocksta = VCUChassisPowerChargeSockSta();
  vcu_chassispowersocfb = VCUChassisPowerSocFb();
  vcu_chassispowervoltfb = VCUChassisPowerVoltFb();
  vcu_chassispowercurrfb = VCUChassisPowerCurrFb();
  vcu_chassisbmsmaxtemp = VCUChassisBmsMaxTemp();
  vcu_chassisbmsreserved_2 = VCUChassisBmsReserved2();
}


// config detail: {'bit': 0, 'description': '预留', 'is_signed_var': False, 'len': 4, 'name': 'VCU_ChassisBmsReserved_1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int V2apowerstafb535::VCUChassisBmsReserved1() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 4, 'description': '车辆充电状态', 'enum': {0: 'VCU_CHASSISPOWERCHARGESTA_NO_CHARGE', 1: 'VCU_CHASSISPOWERCHARGESTA_CHARGE', 2: 'VCU_CHASSISPOWERCHARGESTA_CHARGE_FULL'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisPowerChargeSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2apowerstafb535::VCUChassisPowerChargeSta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 6, 'description': '充电枪连接状态', 'enum': {0: 'VCU_CHASSISPOWERCHARGESOCKSTA_NO_CONNECT', 1: 'VCU_CHASSISPOWERCHARGESOCKSTA_CONNECT'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisPowerChargeSockSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2apowerstafb535::VCUChassisPowerChargeSockSta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(6, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '车辆动力电池电量', 'is_signed_var': False, 'len': 8, 'name': 'VCU_ChassisPowerSocFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int V2apowerstafb535::VCUChassisPowerSocFb() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'description': '车辆动力电池电压', 'is_signed_var': False, 'len': 16, 'name': 'VCU_ChassisPowerVoltFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double V2apowerstafb535::VCUChassisPowerVoltFb() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 32, 'description': '车辆动力电池电流', 'is_signed_var': False, 'len': 16, 'name': 'VCU_ChassisPowerCurrFb', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|1000]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double V2apowerstafb535::VCUChassisPowerCurrFb() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'bit': 48, 'description': 'BMS最高单体温度', 'is_signed_var': False, 'len': 8, 'name': 'VCU_ChassisBmsMaxTemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|80]', 'physical_unit': '℃', 'precision': 1.0, 'type': 'int'}
int V2apowerstafb535::VCUChassisBmsMaxTemp() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 56, 'description': '预留', 'is_signed_var': False, 'len': 8, 'name': 'VCU_ChassisBmsReserved_2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int V2apowerstafb535::VCUChassisBmsReserved2() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

