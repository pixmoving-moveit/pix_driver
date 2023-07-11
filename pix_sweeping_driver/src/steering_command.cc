#include <pix_sweeping_driver/steering_command.hpp>

int32_t SteeringCommand::ID = 0x102;

// public
SteeringCommand::SteeringCommand() { Reset(); }

void SteeringCommand::UpdateData(int check_sum102, int steer_angle_target, int steer_angle_speed, bool steer_en_ctrl) {
  set_p_check_sum102(check_sum102);
  set_p_steer_angle_target(steer_angle_target);
  set_p_steer_angle_speed(steer_angle_speed);
  set_p_steer_en_ctrl(steer_en_ctrl);
}

void SteeringCommand::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * SteeringCommand::get_data()
{
  return data;
}



// config detail: {'bit': 63, 'description': '校验和', 'is_signed_var': False, 'len': 8, 'name': 'check_sum102', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void SteeringCommand::set_p_check_sum102(int check_sum102) {
  // check_sum102 = ProtocolData::BoundedValue(0, 255, check_sum102);
  int x = check_sum102;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
  
}

// config detail: {'bit': 23, 'description': '转向角度', 'is_signed_var': True, 'len': 16, 'name': 'steer_angle_target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
void SteeringCommand::set_p_steer_angle_target(int steer_angle_target) {
  // steer_angle_target = ProtocolData::BoundedValue(-500, 500, steer_angle_target);
  int x = steer_angle_target;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[3] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[2] += to_set1.return_byte_t();
}

// config detail: {'bit': 15, 'description': '转向角速度', 'is_signed_var': True, 'len': 8, 'name': 'steer_angle_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
void SteeringCommand::set_p_steer_angle_speed(int steer_angle_speed) {
  // steer_angle_speed = ProtocolData::BoundedValue(0, 250, steer_angle_speed);
  int x = steer_angle_speed;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 0, 'description': '转向使能', 'is_signed_var': False, 'len': 1, 'name': 'steer_en_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void SteeringCommand::set_p_steer_en_ctrl(bool steer_en_ctrl) {
  int x = steer_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}


