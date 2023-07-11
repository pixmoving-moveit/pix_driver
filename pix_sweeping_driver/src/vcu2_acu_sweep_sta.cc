#include <pix_sweeping_driver/vcu2_acu_sweep_sta.hpp>


Vcu2AcuSweepSta::Vcu2AcuSweepSta() {}

void Vcu2AcuSweepSta::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuSweepSta::Parse() {
  scu_sweep_conntorller_err2_ = scusweepconntorllererr2();
  scu_sweep_conntorller_err1_ = scusweepconntorllererr1();
  scu_sweep_conntroller_current_ = scusweepconntrollercurrent();
  scu_sweep_conntroller_voltage_ = scusweepconntrollervoltage();
  scu_sweep_speed_ = scusweepspeed();
  scu_sweep_travel_mm_ = scusweeptravelmm();
}


// config detail: {'bit': 56, 'description': '扫盘电机电控故障码2', 'is_signed_var': False, 'len': 8, 'name': 'scu_sweep_conntorller_err2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepSta::scusweepconntorllererr2() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'description': '扫盘电机电控故障码1', 'is_signed_var': False, 'len': 8, 'name': 'scu_sweep_conntorller_err1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepSta::scusweepconntorllererr1() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'description': '扫盘电机控制器母线电流', 'is_signed_var': True, 'len': 16, 'name': 'scu_sweep_conntroller_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepSta::scusweepconntrollercurrent() {
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

// config detail: {'bit': 16, 'description': '扫盘电机控制器母线电压', 'is_signed_var': True, 'len': 16, 'name': 'scu_sweep_conntroller_voltage', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepSta::scusweepconntrollervoltage() {
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

// config detail: {'bit': 8, 'description': '扫盘电机实际转速', 'is_signed_var': True, 'len': 8, 'name': 'scu_sweep_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepSta::scusweepspeed() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '扫盘浮动行程值', 'is_signed_var': True, 'len': 8, 'name': 'scu_sweep_travel_mm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepSta::scusweeptravelmm() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

