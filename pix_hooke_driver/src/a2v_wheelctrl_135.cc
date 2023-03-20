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

#include <pix_hooke_driver/a2v_wheelctrl_135.hpp>

int32_t A2vwheelctrl135::ID = 0x135;

// public
A2vwheelctrl135::A2vwheelctrl135() { Reset(); }

void A2vwheelctrl135::UpdateData(double acu_motortorquelfctrl_, double acu_motortorquerfctrl_, double acu_motortorquelrctrl_, double acu_motortorquerrctrl_) {
  set_p_acu_motortorquelfctrl(acu_motortorquelfctrl_);
  set_p_acu_motortorquerfctrl(acu_motortorquerfctrl_);
  set_p_acu_motortorquelrctrl(acu_motortorquelrctrl_);
  set_p_acu_motortorquerrctrl(acu_motortorquerrctrl_);
}

void A2vwheelctrl135::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * A2vwheelctrl135::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'description': '左前电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueLfCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
void A2vwheelctrl135::set_p_acu_motortorquelfctrl(double acu_motortorquelfctrl) {
  // acu_motortorquelfctrl = ProtocolData::BoundedValue(-200.0, 200.0, acu_motortorquelfctrl);
  int x = acu_motortorquelfctrl / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[0] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[1] += to_set1.return_byte_t();
}

// config detail: {'bit': 16, 'description': '右前电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueRfCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
void A2vwheelctrl135::set_p_acu_motortorquerfctrl(double acu_motortorquerfctrl) {
  // acu_motortorquerfctrl = ProtocolData::BoundedValue(-200.0, 200.0, acu_motortorquerfctrl);
  int x = acu_motortorquerfctrl / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[2] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[3] += to_set1.return_byte_t();
}

// config detail: {'bit': 32, 'description': '左后电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueLrCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
void A2vwheelctrl135::set_p_acu_motortorquelrctrl(double acu_motortorquelrctrl) {
  // acu_motortorquelrctrl = ProtocolData::BoundedValue(-200.0, 200.0, acu_motortorquelrctrl);
  int x = acu_motortorquelrctrl / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[4] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[5] += to_set1.return_byte_t();
}

// config detail: {'bit': 48, 'description': '右后电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueRrCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
void A2vwheelctrl135::set_p_acu_motortorquerrctrl(double acu_motortorquerrctrl) {
  // acu_motortorquerrctrl = ProtocolData::BoundedValue(-200.0, 200.0, acu_motortorquerrctrl);
  int x = acu_motortorquerrctrl / 0.100000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[6] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[7] += to_set1.return_byte_t();
}


