#include <pix_robobus_driver/auto_ctrl_msg.hpp>


AutoCtrlMsg::AutoCtrlMsg() {}

void AutoCtrlMsg::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void AutoCtrlMsg::Parse() {
  auto_heartbeat_ = autoheartbeat();
  auto_drive_ctrl_mode_ = autodrivectrlmode();
}


// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'auto_heartbeat', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int AutoCtrlMsg::autoheartbeat() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'auto_drive_ctrl_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int AutoCtrlMsg::autodrivectrlmode() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

