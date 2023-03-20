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

class V2avehicleworkstafb534 {
public:
    static const uint32_t ID = 0x534;
    V2avehicleworkstafb534();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int vcu_drivingmodefb;
    int vcu_chassispowerstafb;
    int vcu_chassispowerdcsta;
    int vcu_chassisspeedlimitedmodefb;
    int vcu_chassispowerlimitesta;
    int vcu_sysecomode;
    double vcu_chassisspeedlimitedvalfb;
    double vcu_chassislowpowervoltsta;
    int vcu_chassisestopstafb;
    int vcu_crashfrontsta;
    int vcu_crashrearsta;
    int vcu_crashleftsta;
    int vcu_crashrightsta;
    int vcu_life;
    int vcu_checksum;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '驾驶模式反馈', 'enum': {0: 'VCU_DRIVINGMODEFB_STANDBY', 1: 'VCU_DRIVINGMODEFB_SELF_DRIVING', 2: 'VCU_DRIVINGMODEFB_REMOTE', 3: 'VCU_DRIVINGMODEFB_MAN'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_DrivingModeFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUDrivingModeFb();

  // config detail: {'bit': 2, 'description': '车辆上电状态反馈', 'enum': {0: 'VCU_CHASSISPOWERSTAFB_INIT', 1: 'VCU_CHASSISPOWERSTAFB_ON_ACC', 2: 'VCU_CHASSISPOWERSTAFB_READY', 3: 'VCU_CHASSISPOWERSTAFB_OFF'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisPowerStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisPowerStaFb();

  // config detail: {'bit': 4, 'description': 'DC工作状态', 'enum': {0: 'VCU_CHASSISPOWERDCSTA_OFF', 1: 'VCU_CHASSISPOWERDCSTA_ON', 2: 'VCU_CHASSISPOWERDCSTA_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisPowerDcSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisPowerDcSta();

  // config detail: {'bit': 8, 'description': '车辆限速状态', 'enum': {0: 'VCU_CHASSISSPEEDLIMITEDMODEFB_DEFAULT', 1: 'VCU_CHASSISSPEEDLIMITEDMODEFB_LIMIT'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisSpeedLimitedModeFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisSpeedLimitedModeFb();

  // config detail: {'bit': 9, 'description': '功率限制状态', 'enum': {0: 'VCU_CHASSISPOWERLIMITESTA_DEFAULT', 1: 'VCU_CHASSISPOWERLIMITESTA_LIMIT'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisPowerLimiteSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisPowerLimiteSta();

  // config detail: {'bit': 10, 'description': '节能模式反馈', 'enum': {0: 'VCU_SYSECOMODE_DEFAULT', 1: 'VCU_SYSECOMODE_ECO', 2: 'VCU_SYSECOMODE_SPORT'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_SysEcoMode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysEcoMode();

  // config detail: {'bit': 16, 'description': '车辆限速值反馈', 'is_signed_var': False, 'len': 16, 'name': 'VCU_ChassisSpeedLimitedValFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|50]', 'physical_unit': 'm/s', 'precision': 0.1, 'type': 'double'}
  double VCUChassisSpeedLimitedValFb();

  // config detail: {'bit': 32, 'description': '低压蓄电池电压', 'is_signed_var': False, 'len': 8, 'name': 'VCU_ChassisLowPowerVoltSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double VCUChassisLowPowerVoltSta();

  // config detail: {'bit': 40, 'description': '紧急停车状态反馈', 'enum': {0: 'VCU_CHASSISESTOPSTAFB_NO', 1: 'VCU_CHASSISESTOPSTAFB_CHASSIS_ESTOP', 2: 'VCU_CHASSISESTOPSTAFB_REMOTE_ESTOP', 3: 'VCU_CHASSISESTOPSTAFB_CHASSIS_ERR_ESTOP'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_ChassisEStopStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUChassisEStopStaFb();

  // config detail: {'bit': 44, 'description': '车辆前碰撞传感器反馈', 'enum': {0: 'VCU_CRASHFRONTSTA_OFF', 1: 'VCU_CRASHFRONTSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashFrontSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUCrashFrontSta();

  // config detail: {'bit': 45, 'description': '车辆后碰撞传感器反馈', 'enum': {0: 'VCU_CRASHREARSTA_OFF', 1: 'VCU_CRASHREARSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashRearSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUCrashRearSta();

  // config detail: {'bit': 46, 'description': '车辆左碰撞传感器反馈', 'enum': {0: 'VCU_CRASHLEFTSTA_OFF', 1: 'VCU_CRASHLEFTSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashLeftSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUCrashLeftSta();

  // config detail: {'bit': 47, 'description': '车辆右碰撞传感器反馈', 'enum': {0: 'VCU_CRASHRIGHTSTA_OFF', 1: 'VCU_CRASHRIGHTSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashRightSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUCrashRightSta();

  // config detail: {'bit': 48, 'description': 'VCU循环计数', 'is_signed_var': False, 'len': 4, 'name': 'VCU_Life', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int VCULife();

  // config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'VCU_CheckSum', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int VCUCheckSum();
};



