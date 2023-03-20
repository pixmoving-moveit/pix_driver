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

class V2avehiclefltsta537 {
public:
    static const uint32_t ID = 0x537;
    V2avehiclefltsta537();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int vcu_sysmotorovertempsta;
    int vcu_sysbmsovertempsta;
    int vcu_sysbrakeovertempsta;
    int vcu_syssteerovertempsta;
    int vcu_sysundervolt;
    int vcu_sysflt;
    int vcu_sysbrakeflt;
    int vcu_sysparkingflt;
    int vcu_syssteerfrontflt;
    int vcu_syssteerbackflt;
    int vcu_sysmotorlfflt;
    int vcu_sysmotorrfflt;
    int vcu_sysmotorlrflt;
    int vcu_sysmotorrrflt;
    int vcu_sysbmsflt;
    int vcu_sysdcflt;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'description': '电机系统过温', 'enum': {0: 'VCU_SYSMOTOROVERTEMPSTA_NORMAL', 1: 'VCU_SYSMOTOROVERTEMPSTA_OVER_TEMP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_SysMotorOverTempSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysMotorOverTempSta();

  // config detail: {'bit': 1, 'description': '电池系统过温', 'enum': {0: 'VCU_SYSBMSOVERTEMPSTA_NORMAL', 1: 'VCU_SYSBMSOVERTEMPSTA_OVER_TEMP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_SysBmsOverTempSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysBmsOverTempSta();

  // config detail: {'bit': 2, 'description': '制动系统过温', 'enum': {0: 'VCU_SYSBRAKEOVERTEMPSTA_NORMAL', 1: 'VCU_SYSBRAKEOVERTEMPSTA_OVER_TEMP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_SysBrakeOverTempSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysBrakeOverTempSta();

  // config detail: {'bit': 3, 'description': '转向系统过温', 'enum': {0: 'VCU_SYSSTEEROVERTEMPSTA_NORMAL', 1: 'VCU_SYSSTEEROVERTEMPSTA_OVER_TEMP'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_SysSteerOverTempSta', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysSteerOverTempSta();

  // config detail: {'bit': 4, 'description': '蓄电池电压过低', 'enum': {0: 'VCU_SYSUNDERVOLT_NORMAL', 1: 'VCU_SYSUNDERVOLT_UNDER_VOLT'}, 'is_signed_var': False, 'len': 1, 'name': 'VCU_SysUnderVolt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysUnderVolt();

  // config detail: {'bit': 8, 'description': '系统故障', 'enum': {0: 'VCU_SYSFLT_NORMAL', 1: 'VCU_SYSFLT_FAULT_LEVEL_1', 2: 'VCU_SYSFLT_FAULT_LEVEL_2', 3: 'VCU_SYSFLT_FAULT_LEVEL_3', 4: 'VCU_SYSFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysFlt();

  // config detail: {'bit': 12, 'description': '制动系统故障', 'enum': {0: 'VCU_SYSBRAKEFLT_NORMAL', 1: 'VCU_SYSBRAKEFLT_FAULT_LEVEL_1', 2: 'VCU_SYSBRAKEFLT_FAULT_LEVEL_2', 3: 'VCU_SYSBRAKEFLT_FAULT_LEVEL_3', 4: 'VCU_SYSBRAKEFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysBrakeFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysBrakeFlt();

  // config detail: {'bit': 16, 'description': '驻车系统故障', 'enum': {0: 'VCU_SYSPARKINGFLT_NORMAL', 1: 'VCU_SYSPARKINGFLT_FAULT_LEVEL_1', 2: 'VCU_SYSPARKINGFLT_FAULT_LEVEL_2', 3: 'VCU_SYSPARKINGFLT_FAULT_LEVEL_3', 4: 'VCU_SYSPARKINGFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysParkingFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysParkingFlt();

  // config detail: {'bit': 20, 'description': '前转向系统故障', 'enum': {0: 'VCU_SYSSTEERFRONTFLT_NORMAL', 1: 'VCU_SYSSTEERFRONTFLT_FAULT_LEVEL_1', 2: 'VCU_SYSSTEERFRONTFLT_FAULT_LEVEL_2', 3: 'VCU_SYSSTEERFRONTFLT_FAULT_LEVEL_3', 4: 'VCU_SYSSTEERFRONTFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysSteerFrontFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysSteerFrontFlt();

  // config detail: {'bit': 24, 'description': '后转向系统故障', 'enum': {0: 'VCU_SYSSTEERBACKFLT_NORMAL', 1: 'VCU_SYSSTEERBACKFLT_FAULT_LEVEL_1', 2: 'VCU_SYSSTEERBACKFLT_FAULT_LEVEL_2', 3: 'VCU_SYSSTEERBACKFLT_FAULT_LEVEL_3', 4: 'VCU_SYSSTEERBACKFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysSteerBackFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysSteerBackFlt();

  // config detail: {'bit': 28, 'description': '左前电机系统故障', 'enum': {0: 'VCU_SYSMOTORLFFLT_NORMAL', 1: 'VCU_SYSMOTORLFFLT_FAULT_LEVEL_1', 2: 'VCU_SYSMOTORLFFLT_FAULT_LEVEL_2', 3: 'VCU_SYSMOTORLFFLT_FAULT_LEVEL_3', 4: 'VCU_SYSMOTORLFFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysMotorLfFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysMotorLfFlt();

  // config detail: {'bit': 32, 'description': '右前电机系统故障', 'enum': {0: 'VCU_SYSMOTORRFFLT_NORMAL', 1: 'VCU_SYSMOTORRFFLT_FAULT_LEVEL_1', 2: 'VCU_SYSMOTORRFFLT_FAULT_LEVEL_2', 3: 'VCU_SYSMOTORRFFLT_FAULT_LEVEL_3', 4: 'VCU_SYSMOTORRFFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysMotorRfFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysMotorRfFlt();

  // config detail: {'bit': 36, 'description': '左后电机系统故障', 'enum': {0: 'VCU_SYSMOTORLRFLT_NORMAL', 1: 'VCU_SYSMOTORLRFLT_FAULT_LEVEL_1', 2: 'VCU_SYSMOTORLRFLT_FAULT_LEVEL_2', 3: 'VCU_SYSMOTORLRFLT_FAULT_LEVEL_3', 4: 'VCU_SYSMOTORLRFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysMotorLrFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysMotorLrFlt();

  // config detail: {'bit': 40, 'description': '右后电机系统故障', 'enum': {0: 'VCU_SYSMOTORRRFLT_NORMAL', 1: 'VCU_SYSMOTORRRFLT_FAULT_LEVEL_1', 2: 'VCU_SYSMOTORRRFLT_FAULT_LEVEL_2', 3: 'VCU_SYSMOTORRRFLT_FAULT_LEVEL_3', 4: 'VCU_SYSMOTORRRFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysMotorRrFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysMotorRrFlt();

  // config detail: {'bit': 44, 'description': 'BMS系统故障', 'enum': {0: 'VCU_SYSBMSFLT_NORMAL', 1: 'VCU_SYSBMSFLT_FAULT_LEVEL_1', 2: 'VCU_SYSBMSFLT_FAULT_LEVEL_2', 3: 'VCU_SYSBMSFLT_FAULT_LEVEL_3', 4: 'VCU_SYSBMSFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysBmsFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysBmsFlt();

  // config detail: {'bit': 48, 'description': 'DC系统故障', 'enum': {0: 'VCU_SYSDCFLT_NORMAL', 1: 'VCU_SYSDCFLT_FAULT_LEVEL_1', 2: 'VCU_SYSDCFLT_FAULT_LEVEL_2', 3: 'VCU_SYSDCFLT_FAULT_LEVEL_3', 4: 'VCU_SYSDCFLT_FAULT_LEVEL_4'}, 'is_signed_var': False, 'len': 4, 'name': 'VCU_SysDcFlt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int VCUSysDcFlt();
};



