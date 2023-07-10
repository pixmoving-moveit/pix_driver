#include <pix_sweeping_driver/vcu2_acu_sweep_fan_sta.hpp>


Vcu2AcuSweepFanSta::Vcu2AcuSweepFanSta() {}

void Vcu2AcuSweepFanSta::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuSweepFanSta::Parse() {
  scu_fan_conntorller_err2_ = scufanconntorllererr2();
  scu_fan_conntorller_err1_ = scufanconntorllererr1();
  scu_fan_conntorller_temp_ = scufanconntorllertemp();
  scu_fan_motor_temp_ = scufanmotortemp();
  scu_fan_conntorller_current_ = scufanconntorllercurrent();
  scu_fan_conntorller_main_voltage_ = scufanconntorllermainvoltage();
}


// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'scu_fan_conntorller_err2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepFanSta::scufanconntorllererr2() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'scu_fan_conntorller_err1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepFanSta::scufanconntorllererr1() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name': 'scu_fan_conntorller_temp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|160]', 'physical_unit': '℃', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepFanSta::scufanconntorllertemp() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': True, 'len': 8, 'name': 'scu_fan_motor_temp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|210]', 'physical_unit': '℃', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepFanSta::scufanmotortemp() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'scu_fan_conntorller_current', 'offset': -2000.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'A', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepFanSta::scufanconntorllercurrent() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x + -2000.000000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'scu_fan_conntorller_main_voltage', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|500]', 'physical_unit': 'V', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuSweepFanSta::scufanconntorllermainvoltage() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

