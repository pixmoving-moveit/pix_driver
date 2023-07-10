#include <pix_sweeping_driver/acu_sweep_ctrl_cmd.hpp>

int32_t AcuSweepCtrlCmd::ID = 0x107;

// public
AcuSweepCtrlCmd::AcuSweepCtrlCmd() { Reset(); }

void AcuSweepCtrlCmd::UpdateData(int fan_speed_ctrl, int mowing_speed_ctrl, int sweep_mode_ctrl, bool fan_speed_mode, int fan_mode_ctrl, int sweep_plate_up_down, int mouthpiece_up_down_ctrl, bool shaker_duster_ctrl, bool dedusting_ctrl, int auto_garbage_dump_start_ctrl, int auto_cleaning_start_ctrl) {
  set_p_fan_speed_ctrl(fan_speed_ctrl);
  set_p_mowing_speed_ctrl(mowing_speed_ctrl);
  set_p_sweep_mode_ctrl(sweep_mode_ctrl);
  set_p_fan_speed_mode(fan_speed_mode);
  set_p_fan_mode_ctrl(fan_mode_ctrl);
  set_p_sweep_plate_up_down(sweep_plate_up_down);
  set_p_mouthpiece_up_down_ctrl(mouthpiece_up_down_ctrl);
  set_p_shaker_duster_ctrl(shaker_duster_ctrl);
  set_p_dedusting_ctrl(dedusting_ctrl);
  set_p_auto_garbage_dump_start_ctrl(auto_garbage_dump_start_ctrl);
  set_p_auto_cleaning_start_ctrl(auto_cleaning_start_ctrl);
}

void AcuSweepCtrlCmd::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * AcuSweepCtrlCmd::get_data()
{
  return data;
}



// config detail: {'bit': 47, 'is_signed_var': True, 'len': 16, 'name': 'fan_speed_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_fan_speed_ctrl(int fan_speed_ctrl) {
  // fan_speed_ctrl = ProtocolData::BoundedValue(0, 65535, fan_speed_ctrl);
  int x = fan_speed_ctrl;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[6] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[5] += to_set1.return_byte_t();
}

// config detail: {'bit': 25, 'is_signed_var': False, 'len': 2, 'name': 'mowing_speed_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_mowing_speed_ctrl(int mowing_speed_ctrl) {
  // mowing_speed_ctrl = ProtocolData::BoundedValue(0, 3, mowing_speed_ctrl);
  int x = mowing_speed_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[3] += to_set.return_byte_t();
  
}

// config detail: {'bit': 21, 'is_signed_var': False, 'len': 2, 'name': 'sweep_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_sweep_mode_ctrl(int sweep_mode_ctrl) {
  // sweep_mode_ctrl = ProtocolData::BoundedValue(0, 3, sweep_mode_ctrl);
  int x = sweep_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 2);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 19, 'is_signed_var': False, 'len': 1, 'name': 'fan_speed_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void AcuSweepCtrlCmd::set_p_fan_speed_mode(bool fan_speed_mode) {
  int x = fan_speed_mode;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 3, 1);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 17, 'is_signed_var': False, 'len': 2, 'name': 'fan_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_fan_mode_ctrl(int fan_mode_ctrl) {
  // fan_mode_ctrl = ProtocolData::BoundedValue(0, 3, fan_mode_ctrl);
  int x = fan_mode_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[2] += to_set.return_byte_t();
  
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 2, 'name': 'sweep_plate_up_down', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_sweep_plate_up_down(int sweep_plate_up_down) {
  // sweep_plate_up_down = ProtocolData::BoundedValue(0, 3, sweep_plate_up_down);
  int x = sweep_plate_up_down;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 6, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 13, 'is_signed_var': False, 'len': 2, 'name': 'mouthpiece_up_down_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_mouthpiece_up_down_ctrl(int mouthpiece_up_down_ctrl) {
  // mouthpiece_up_down_ctrl = ProtocolData::BoundedValue(0, 3, mouthpiece_up_down_ctrl);
  int x = mouthpiece_up_down_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'shaker_duster_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void AcuSweepCtrlCmd::set_p_shaker_duster_ctrl(bool shaker_duster_ctrl) {
  int x = shaker_duster_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 1, 1);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'dedusting_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void AcuSweepCtrlCmd::set_p_dedusting_ctrl(bool dedusting_ctrl) {
  int x = dedusting_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 3, 'is_signed_var': False, 'len': 2, 'name': 'auto_garbage_dump_start_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_auto_garbage_dump_start_ctrl(int auto_garbage_dump_start_ctrl) {
  // auto_garbage_dump_start_ctrl = ProtocolData::BoundedValue(0, 3, auto_garbage_dump_start_ctrl);
  int x = auto_garbage_dump_start_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 2);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 2, 'name': 'auto_cleaning_start_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void AcuSweepCtrlCmd::set_p_auto_cleaning_start_ctrl(int auto_cleaning_start_ctrl) {
  // auto_cleaning_start_ctrl = ProtocolData::BoundedValue(0, 3, auto_cleaning_start_ctrl);
  int x = auto_cleaning_start_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[0] += to_set.return_byte_t();
  
}

