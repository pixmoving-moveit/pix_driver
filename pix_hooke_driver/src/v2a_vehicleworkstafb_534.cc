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

#include <pix_hooke_driver/v2a_vehicleworkstafb_534.hpp>


V2avehicleworkstafb534::V2avehicleworkstafb534() {}

void V2avehicleworkstafb534::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2avehicleworkstafb534::Parse() {
  vcu_drivingmodefb = VCUDrivingModeFb();
  vcu_chassispowerstafb = VCUChassisPowerStaFb();
  vcu_chassispowerdcsta = VCUChassisPowerDcSta();
  vcu_chassisspeedlimitedmodefb = VCUChassisSpeedLimitedModeFb();
  vcu_chassispowerlimitesta = VCUChassisPowerLimiteSta();
  vcu_sysecomode = VCUSysEcoMode();
  vcu_chassisspeedlimitedvalfb = VCUChassisSpeedLimitedValFb();
  vcu_chassislowpowervoltsta = VCUChassisLowPowerVoltSta();
  vcu_chassisestopstafb = VCUChassisEStopStaFb();
  vcu_crashfrontsta = VCUCrashFrontSta();
  vcu_crashrearsta = VCUCrashRearSta();
  vcu_crashleftsta = VCUCrashLeftSta();
  vcu_crashrightsta = VCUCrashRightSta();
  vcu_life = VCULife();
  vcu_checksum = VCUCheckSum();
}


// config detail: {'bit': 0, 'description': '驾驶模式反馈', 'enum': {0: 'VCU_DRIVINGMODEFB_STANDBY', 1: 'VCU_DRIVINGMODEFB_SELF_DRIVING', 2: 'VCU_DRIVINGMODEFB_REMOTE', 3: 'VCU_DRIVINGMODEFB_MAN'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_DrivingModeFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUDrivingModeFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 2, 'description': '车辆上电状态反馈', 'enum': {0: 'VCU_CHASSISPOWERSTAFB_INIT', 1: 'VCU_CHASSISPOWERSTAFB_ON_ACC', 2: 'VCU_CHASSISPOWERSTAFB_READY', 3: 'VCU_CHASSISPOWERSTAFB_OFF'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisPowerStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUChassisPowerStaFb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 4, 'description': 'DC工作状态', 'enum': {0: 'VCU_CHASSISPOWERDCSTA_OFF', 1: 'VCU_CHASSISPOWERDCSTA_ON', 2: 'VCU_CHASSISPOWERDCSTA_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_ChassisPowerDcSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUChassisPowerDcSta() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '车辆限速状态', 'enum': {0: 'VCU_CHASSISSPEEDLIMITEDMODEFB_DEFAULT', 1: 'VCU_CHASSISSPEEDLIMITEDMODEFB_LIMIT'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisSpeedLimitedModeFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUChassisSpeedLimitedModeFb() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 9, 'description': '功率限制状态', 'enum': {0: 'VCU_CHASSISPOWERLIMITESTA_DEFAULT', 1: 'VCU_CHASSISPOWERLIMITESTA_LIMIT'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ChassisPowerLimiteSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUChassisPowerLimiteSta() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(1, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 10, 'description': '节能模式反馈', 'enum': {0: 'VCU_SYSECOMODE_DEFAULT', 1: 'VCU_SYSECOMODE_ECO', 2: 'VCU_SYSECOMODE_SPORT'}, 'is_signed_var': False, 'len': 2, 'name': 'VCU_SysEcoMode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUSysEcoMode() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(2, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 16, 'description': '车辆限速值反馈', 'is_signed_var': False, 'len': 16, 'name': 'VCU_ChassisSpeedLimitedValFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|50]', 'physical_unit': 'm/s', 'precision': 0.1, 'type': 'double'}
double V2avehicleworkstafb534::VCUChassisSpeedLimitedValFb() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 32, 'description': '低压蓄电池电压', 'is_signed_var': False, 'len': 8, 'name': 'VCU_ChassisLowPowerVoltSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double V2avehicleworkstafb534::VCUChassisLowPowerVoltSta() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 40, 'description': '紧急停车状态反馈', 'enum': {0: 'VCU_CHASSISESTOPSTAFB_NO', 1: 'VCU_CHASSISESTOPSTAFB_CHASSIS_ESTOP', 2: 'VCU_CHASSISESTOPSTAFB_REMOTE_ESTOP', 3: 'VCU_CHASSISESTOPSTAFB_CHASSIS_ERR_ESTOP'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_ChassisEStopStaFb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUChassisEStopStaFb() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 4);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 44, 'description': '车辆前碰撞传感器反馈', 'enum': {0: 'VCU_CRASHFRONTSTA_OFF', 1: 'VCU_CRASHFRONTSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashFrontSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUCrashFrontSta() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(4, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 45, 'description': '车辆后碰撞传感器反馈', 'enum': {0: 'VCU_CRASHREARSTA_OFF', 1: 'VCU_CRASHREARSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashRearSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUCrashRearSta() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(5, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 46, 'description': '车辆左碰撞传感器反馈', 'enum': {0: 'VCU_CRASHLEFTSTA_OFF', 1: 'VCU_CRASHLEFTSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashLeftSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUCrashLeftSta() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(6, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 47, 'description': '车辆右碰撞传感器反馈', 'enum': {0: 'VCU_CRASHRIGHTSTA_OFF', 1: 'VCU_CRASHRIGHTSTA_COLLIDE'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_CrashRightSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int V2avehicleworkstafb534::VCUCrashRightSta() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(7, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 48, 'description': 'VCU循环计数', 'is_signed_var': False, 'len': 4, 'name': 'VCU_Life', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int V2avehicleworkstafb534::VCULife() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'description': '校验sum=byte0 xor byte1 xor...byte6', 'is_signed_var': False, 'len': 8, 'name': 'VCU_CheckSum', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int V2avehicleworkstafb534::VCUCheckSum() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

