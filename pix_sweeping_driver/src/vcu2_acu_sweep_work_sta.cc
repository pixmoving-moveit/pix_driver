#include <pix_sweeping_driver/vcu2_acu_sweep_work_sta.hpp>


Vcu2AcuSweepWorkSta::Vcu2AcuSweepWorkSta() {}

void Vcu2AcuSweepWorkSta::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuSweepWorkSta::Parse() {
  sweep_emergency_sig_fb_ = sweepemergencysigfb();
  vcu_sweep_plate_up_down_sta_fb_ = vcusweepplateupdownstafb();
  vcu_auto_garbage_dump_sta_fb_ = vcuautogarbagedumpstafb();
  vcu_auto_cleaning_sta_fb_ = vcuautocleaningstafb();
}


// config detail: {'bit': 10, 'description': '清扫系统紧急停止反馈', 'is_signed_var': True, 'len': 1, 'name': 'sweep_emergency_sig_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepWorkSta::sweepemergencysigfb() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(2, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 9, 'description': '扫盘升降状态反馈', 'enum': {0: 'NO', 1: 'UP', 2: 'DOWN'}, 'is_signed_var': True, 'len': 2, 'name': 'vcu_sweep_plate_up_down_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcu2AcuSweepWorkSta::vcusweepplateupdownstafb() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 2);

  x <<= 30;
  x >>= 30;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 1, 'description': '一键倾倒状态反馈', 'is_signed_var': True, 'len': 1, 'name': 'vcu_auto_garbage_dump_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepWorkSta::vcuautogarbagedumpstafb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '一键清扫状态反馈', 'is_signed_var': True, 'len': 1, 'name': 'vcu_auto_cleaning_sta_fb', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcu2AcuSweepWorkSta::vcuautocleaningstafb() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

