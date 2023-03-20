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

#include <pix_hooke_driver/v2a_vehiclestafb_536.hpp>


V2avehiclestafb536::V2avehiclestafb536() {}

void V2avehiclestafb536::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2avehiclestafb536::Parse() {
  vcu_vehicleposlampfb = VCUVehiclePosLampFb();
  vcu_vehicleheadlampfb = VCUVehicleHeadLampFb();
  vcu_vehicleleftlampfb = VCUVehicleLeftLampFb();
  vcu_vehiclerightlampfb = VCUVehicleRightLampFb();
  vcu_vehiclehighbeamfb = VCUVehicleHighBeamFb();
  vcu_vehiclefoglampfb = VCUVehicleFogLampFb();
  vcu_vehiclehazardwarlampfb = VCUVehicleHazardWarLampFb();
  vcu_vehiclebodylampfb = VCUVehicleBodyLampFb();
  vcu_vehiclereadlampfb = VCUVehicleReadLampFb();
  acu_vehiclewindowfb = ACUVehicleWindowFb();
  vcu_vehicledoorstafb = VCUVehicleDoorStaFb();
  vcu_vehiclewipersstafb = VCUVehicleWipersStaFb();
  vcu_vehiclesafetybelt1 = VCUVehicleSafetyBelt1();
  vcu_vehiclesafetybelt2 = VCUVehicleSafetyBelt2();
  vcu_vehiclesafetybelt3 = VCUVehicleSafetyBelt3();
  vcu_vehiclesafetybelt4 = VCUVehicleSafetyBelt4();
}


// config detail: {'bit': 0, 'description': '位置灯状态反馈', 'enum': {0: 'VCU_VEHICLEPOSLAMPFB_OFF', 1: 'VCU_VEHICLEPOSLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehiclePosLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehiclePosLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 1, 'description': '近光灯状态反馈', 'enum': {0: 'VCU_VEHICLEHEADLAMPFB_OFF', 1: 'VCU_VEHICLEHEADLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleHeadLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleHeadLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 2, 'description': '左转向灯状态反馈', 'enum': {0: 'VCU_VEHICLELEFTLAMPFB_OFF', 1: 'VCU_VEHICLELEFTLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleLeftLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleLeftLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 3, 'description': '右转向灯状态反馈', 'enum': {0: 'VCU_VEHICLERIGHTLAMPFB_OFF', 1: 'VCU_VEHICLERIGHTLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleRightLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleRightLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(3, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 4, 'description': '远光灯状态反馈', 'enum': {0: 'VCU_VEHICLEHIGHBEAMFB_OFF', 1: 'VCU_VEHICLEHIGHBEAMFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleHighBeamFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleHighBeamFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 5, 'description': '雾灯状态反馈', 'enum': {0: 'VCU_VEHICLEFOGLAMPFB_OFF', 1: 'VCU_VEHICLEFOGLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleFogLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleFogLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(5, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 6, 'description': '危险警示灯开关状态', 'enum': {0: 'VCU_VEHICLEHAZARDWARLAMPFB_OFF', 1: 'VCU_VEHICLEHAZARDWARLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleHazardWarLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleHazardWarLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(6, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 7, 'description': '车外氛围灯状态反馈', 'enum': {0: 'VCU_VEHICLEBODYLAMPFB_OFF', 1: 'VCU_VEHICLEBODYLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleBodyLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleBodyLampFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(7, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '车内灯光状态反馈', 'enum': {0: 'VCU_VEHICLEREADLAMPFB_OFF', 1: 'VCU_VEHICLEREADLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleReadLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleReadLampFb() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 9, 'description': '车内车窗状态反馈（预留）', 'enum': {0: 'ACU_VEHICLEWINDOWFB_INVALID', 1: 'ACU_VEHICLEWINDOWFB_OPENING', 2: 'ACU_VEHICLEWINDOWFB_CLOSING', 3: 'ACU_VEHICLEWINDOWFB_OPEN_INPLACE', 4: 'ACU_VEHICLEWINDOWFB_CLOSE_INPLACE', 5: 'ACU_VEHICLEWINDOWFB_OPEN_TIMEOUT', 6: 'ACU_VEHICLEWINDOWFB_CLOSE_TIMEOUT'}, 'is_signed_var': False, 'len': 4, 'name': 'ACU_VehicleWindowFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::ACUVehicleWindowFb() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(1, 4);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 16, 'description': '车门状态反馈', 'enum': {0: 'VCU_VEHICLEDOORSTAFB_INVALID', 1: 'VCU_VEHICLEDOORSTAFB_OPENING', 2: 'VCU_VEHICLEDOORSTAFB_CLOSING', 3: 'VCU_VEHICLEDOORSTAFB_OPEN_INPLACE', 4: 'VCU_VEHICLEDOORSTAFB_CLOSE_INPLACE', 5: 'VCU_VEHICLEDOORSTAFB_OPEN_TIMEOUT', 6: 'VCU_VEHICLEDOORSTAFB_CLOSE_TIMEOUT'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_VehicleDoorStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleDoorStaFb() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 4);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 20, 'description': '雨刮状态反馈', 'enum': {0: 'VCU_VEHICLEWIPERSSTAFB_OFF', 1: 'VCU_VEHICLEWIPERSSTAFB_LOW', 2: 'VCU_VEHICLEWIPERSSTAFB_MID', 3: 'VCU_VEHICLEWIPERSSTAFB_HIGH'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleWipersStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleWipersStaFb() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(4, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 24, 'description': '安全带状态1反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT1_NO', 1: 'VCU_VEHICLESAFETYBELT1_SIT', 2: 'VCU_VEHICLESAFETYBELT1_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT1_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleSafetyBelt1() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 26, 'description': '安全带状态2反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT2_NO', 1: 'VCU_VEHICLESAFETYBELT2_SIT', 2: 'VCU_VEHICLESAFETYBELT2_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT2_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleSafetyBelt2() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(2, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 28, 'description': '安全带状态3反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT3_NO', 1: 'VCU_VEHICLESAFETYBELT3_SIT', 2: 'VCU_VEHICLESAFETYBELT3_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT3_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt3', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleSafetyBelt3() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(4, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 30, 'description': '安全带状态4反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT4_NO', 1: 'VCU_VEHICLESAFETYBELT4_SIT', 2: 'VCU_VEHICLESAFETYBELT4_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT4_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt4', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehiclestafb536::VCUVehicleSafetyBelt4() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(6, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

