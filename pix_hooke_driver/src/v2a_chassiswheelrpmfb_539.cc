#include <pix_hooke_driver/v2a_chassiswheelrpmfb_539.hpp>


V2achassiswheelrpmfb539::V2achassiswheelrpmfb539() {}

void V2achassiswheelrpmfb539::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void V2achassiswheelrpmfb539::Parse() {
  vcu_chassiswheelrpmlf = VCUChassisWheelRpmLf();
  vcu_chassiswheelrpmrf = VCUChassisWheelRpmRf();
  vcu_chassiswheelrpmlr = VCUChassisWheelRpmLr();
  vcu_chassiswheelrpmrr = VCUChassisWheelRpmRr();
}


// config detail: {'bit': 0, 'description': '左前轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmLf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int V2achassiswheelrpmfb539::VCUChassisWheelRpmLf() {
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

// config detail: {'bit': 16, 'description': '右前轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmRf', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int V2achassiswheelrpmfb539::VCUChassisWheelRpmRf() {
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

// config detail: {'bit': 32, 'description': '左后轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmLr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int V2achassiswheelrpmfb539::VCUChassisWheelRpmLr() {
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

// config detail: {'bit': 48, 'description': '右后轮转速', 'is_signed_var': True, 'len': 16, 'name': 'VCU_ChassisWheelRpmRr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-2000|2000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int V2achassiswheelrpmfb539::VCUChassisWheelRpmRr() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

