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

class V2avehiclestafb536 {
public:
    static const uint32_t ID = 0x536;
    V2avehiclestafb536();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int vcu_vehicleposlampfb;
    int vcu_vehicleheadlampfb;
    int vcu_vehicleleftlampfb;
    int vcu_vehiclerightlampfb;
    int vcu_vehiclehighbeamfb;
    int vcu_vehiclefoglampfb;
    int vcu_vehiclehazardwarlampfb;
    int vcu_vehiclebodylampfb;
    int vcu_vehiclereadlampfb;
    int acu_vehiclewindowfb;
    int vcu_vehicledoorstafb;
    int vcu_vehiclewipersstafb;
    int vcu_vehiclesafetybelt1;
    int vcu_vehiclesafetybelt2;
    int vcu_vehiclesafetybelt3;
    int vcu_vehiclesafetybelt4;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '位置灯状态反馈', 'enum': {0: 'VCU_VEHICLEPOSLAMPFB_OFF', 1: 'VCU_VEHICLEPOSLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehiclePosLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehiclePosLampFb();

  // config detail: {'bit': 1, 'description': '近光灯状态反馈', 'enum': {0: 'VCU_VEHICLEHEADLAMPFB_OFF', 1: 'VCU_VEHICLEHEADLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleHeadLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleHeadLampFb();

  // config detail: {'bit': 2, 'description': '左转向灯状态反馈', 'enum': {0: 'VCU_VEHICLELEFTLAMPFB_OFF', 1: 'VCU_VEHICLELEFTLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleLeftLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleLeftLampFb();

  // config detail: {'bit': 3, 'description': '右转向灯状态反馈', 'enum': {0: 'VCU_VEHICLERIGHTLAMPFB_OFF', 1: 'VCU_VEHICLERIGHTLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleRightLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleRightLampFb();

  // config detail: {'bit': 4, 'description': '远光灯状态反馈', 'enum': {0: 'VCU_VEHICLEHIGHBEAMFB_OFF', 1: 'VCU_VEHICLEHIGHBEAMFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleHighBeamFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleHighBeamFb();

  // config detail: {'bit': 5, 'description': '雾灯状态反馈', 'enum': {0: 'VCU_VEHICLEFOGLAMPFB_OFF', 1: 'VCU_VEHICLEFOGLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleFogLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleFogLampFb();

  // config detail: {'bit': 6, 'description': '危险警示灯开关状态', 'enum': {0: 'VCU_VEHICLEHAZARDWARLAMPFB_OFF', 1: 'VCU_VEHICLEHAZARDWARLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleHazardWarLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleHazardWarLampFb();

  // config detail: {'bit': 7, 'description': '车外氛围灯状态反馈', 'enum': {0: 'VCU_VEHICLEBODYLAMPFB_OFF', 1: 'VCU_VEHICLEBODYLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleBodyLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleBodyLampFb();

  // config detail: {'bit': 8, 'description': '车内灯光状态反馈', 'enum': {0: 'VCU_VEHICLEREADLAMPFB_OFF', 1: 'VCU_VEHICLEREADLAMPFB_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_VehicleReadLampFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleReadLampFb();

  // config detail: {'bit': 9, 'description': '车内车窗状态反馈（预留）', 'enum': {0: 'ACU_VEHICLEWINDOWFB_INVALID', 1: 'ACU_VEHICLEWINDOWFB_OPENING', 2: 'ACU_VEHICLEWINDOWFB_CLOSING', 3: 'ACU_VEHICLEWINDOWFB_OPEN_INPLACE', 4: 'ACU_VEHICLEWINDOWFB_CLOSE_INPLACE', 5: 'ACU_VEHICLEWINDOWFB_OPEN_TIMEOUT', 6: 'ACU_VEHICLEWINDOWFB_CLOSE_TIMEOUT'}, 'is_signed_var': False, 'len': 4, 'name': 'ACU_VehicleWindowFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int ACUVehicleWindowFb();

  // config detail: {'bit': 16, 'description': '车门状态反馈', 'enum': {0: 'VCU_VEHICLEDOORSTAFB_INVALID', 1: 'VCU_VEHICLEDOORSTAFB_OPENING', 2: 'VCU_VEHICLEDOORSTAFB_CLOSING', 3: 'VCU_VEHICLEDOORSTAFB_OPEN_INPLACE', 4: 'VCU_VEHICLEDOORSTAFB_CLOSE_INPLACE', 5: 'VCU_VEHICLEDOORSTAFB_OPEN_TIMEOUT', 6: 'VCU_VEHICLEDOORSTAFB_CLOSE_TIMEOUT'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_VehicleDoorStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleDoorStaFb();

  // config detail: {'bit': 20, 'description': '雨刮状态反馈', 'enum': {0: 'VCU_VEHICLEWIPERSSTAFB_OFF', 1: 'VCU_VEHICLEWIPERSSTAFB_LOW', 2: 'VCU_VEHICLEWIPERSSTAFB_MID', 3: 'VCU_VEHICLEWIPERSSTAFB_HIGH'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleWipersStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleWipersStaFb();

  // config detail: {'bit': 24, 'description': '安全带状态1反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT1_NO', 1: 'VCU_VEHICLESAFETYBELT1_SIT', 2: 'VCU_VEHICLESAFETYBELT1_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT1_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleSafetyBelt1();

  // config detail: {'bit': 26, 'description': '安全带状态2反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT2_NO', 1: 'VCU_VEHICLESAFETYBELT2_SIT', 2: 'VCU_VEHICLESAFETYBELT2_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT2_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleSafetyBelt2();

  // config detail: {'bit': 28, 'description': '安全带状态3反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT3_NO', 1: 'VCU_VEHICLESAFETYBELT3_SIT', 2: 'VCU_VEHICLESAFETYBELT3_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT3_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt3', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleSafetyBelt3();

  // config detail: {'bit': 30, 'description': '安全带状态4反馈', 'enum': {0: 'VCU_VEHICLESAFETYBELT4_NO', 1: 'VCU_VEHICLESAFETYBELT4_SIT', 2: 'VCU_VEHICLESAFETYBELT4_SAFETYBELT', 3: 'VCU_VEHICLESAFETYBELT4_UNKNOW_传感器故障'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_VehicleSafetyBelt4', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUVehicleSafetyBelt4();
};



