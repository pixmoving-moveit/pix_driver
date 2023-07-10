#include <pix_sweeping_driver/vcu2_acu_chassis_err_code2.hpp>


Vcu2AcuChassisErrCode2::Vcu2AcuChassisErrCode2() {}

void Vcu2AcuChassisErrCode2::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcu2AcuChassisErrCode2::Parse() {
  chassis_ebs_error_code3_ = chassisebserrorcode3();
  chassis_bms_error_code_ = chassisbmserrorcode();
  chassis_back_eps_error_code_ = chassisbackepserrorcode();
  chassis_front_eps_error_code_ = chassisfrontepserrorcode();
  chassis_ebs_error_code2_ = chassisebserrorcode2();
  chassis_ebs_error_code1_ = chassisebserrorcode1();
  chassis_ebs_errorlv_ = chassisebserrorlv();
}


// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisebserrorcode3() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'chassis_bms_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisbmserrorcode() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'chassis_back_eps_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisbackepserrorcode() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 8));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'chassis_front_eps_error_code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisfrontepserrorcode() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 7));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisebserrorcode2() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 5));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'chassis_ebs_error_code1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisebserrorcode1() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 4, 'name': 'chassis_ebs_errorlv', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcu2AcuChassisErrCode2::chassisebserrorlv() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 1);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  int ret = x;
  return ret;
}

