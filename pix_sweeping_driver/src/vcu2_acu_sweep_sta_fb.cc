#include <pix_sweeping_driver/vcu2_acu_sweep_sta_fb.hpp>


Vcu2AcuSweepStaFb::Vcu2AcuSweepStaFb() {}

void Vcu2AcuSweepStaFb::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuSweepStaFb::Parse() {
  scu_mowing_speed_fb_ = scumowingspeedfb();
  push_rod_travel_difference_alarm_ = pushrodtraveldifferencealarm();
  end_of_garbage_dumping_des_ = endofgarbagedumpingdes();
  end_of_garbage_dumping_ = endofgarbagedumping();
  scu_fan_speed_fb_ = scufanspeedfb();
  scu_liquid_leve_ = sculiquidleve();
  scu_heartbeat_ = scuheartbeat();
  scan_controller_communication_fault_ = scancontrollercommunicationfault();
  fan_controller_communication_fault_ = fancontrollercommunicationfault();
  scu_filter_clogging_ = scufilterclogging();
  scu_sweep_life_end_ = scusweeplifeend();
  scu_dustbin_full_ = scudustbinfull();
  scu_liquid_leve_high_ = sculiquidlevehigh();
  scu_liquid_level_low_ = sculiquidlevellow();
}


// config detail: {'bit': 32, 'description': '割草电机实际转速', 'is_signed_var': True, 'len': 16, 'name': 'scu_mowing_speed_fb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepStaFb::scumowingspeedfb() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'description': '推杆行程差值报警', 'is_signed_var': False, 'len': 2, 'name': 'push_rod_travel_difference_alarm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepStaFb::pushrodtraveldifferencealarm() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 49, 'description': '一键垃圾倾倒下降结束', 'is_signed_var': False, 'len': 1, 'name': 'end_of_garbage_dumping_des', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::endofgarbagedumpingdes() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 48, 'description': '垃圾倾倒倾翻结束', 'is_signed_var': False, 'len': 1, 'name': 'end_of_garbage_dumping', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::endofgarbagedumping() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 16, 'description': '风机电机实际转速', 'is_signed_var': True, 'len': 16, 'name': 'scu_fan_speed_fb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepStaFb::scufanspeedfb() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'description': '清水箱液位值', 'is_signed_var': False, 'len': 8, 'name': 'scu_liquid_leve', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepStaFb::sculiquidleve() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'description': '上装控制器心跳', 'is_signed_var': False, 'len': 1, 'name': 'scu_heartbeat', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::scuheartbeat() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 6, 'description': '扫盘控制器通信故障报警', 'is_signed_var': False, 'len': 1, 'name': 'scan_controller_communication_fault', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::scancontrollercommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 5, 'description': '风机控制器通信故障报警', 'is_signed_var': False, 'len': 1, 'name': 'fan_controller_communication_fault', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::fancontrollercommunicationfault() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 4, 'description': '滤芯堵赛报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_filter_clogging', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::scufilterclogging() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 3, 'description': '扫盘磨损结束报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_sweep_life_end', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::scusweeplifeend() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 2, 'description': '垃圾箱满溢报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_dustbin_full', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::scudustbinfull() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 1, 'description': '清水箱高液位报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_liquid_leve_high', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::sculiquidlevehigh() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '清水箱低液位报警', 'is_signed_var': False, 'len': 1, 'name': 'scu_liquid_level_low', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepStaFb::sculiquidlevellow() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

