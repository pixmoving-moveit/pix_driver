#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class AcuSweepCtrlCmd {
public:
	static  int32_t ID;

	AcuSweepCtrlCmd();

  	void UpdateData(int fan_speed_ctrl, int mowing_speed_ctrl, int sweep_mode_ctrl, bool fan_speed_mode, int fan_mode_ctrl, int sweep_plate_up_down, int mouthpiece_up_down_ctrl, bool shaker_duster_ctrl, bool dedusting_ctrl, int auto_garbage_dump_start_ctrl, int auto_cleaning_start_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 47, 'is_signed_var': True, 'len': 16, 'name': 'fan_speed_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  void set_p_fan_speed_ctrl(int fan_speed_ctrl);

  // config detail: {'bit': 25, 'is_signed_var': False, 'len': 2, 'name': 'mowing_speed_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_mowing_speed_ctrl(int mowing_speed_ctrl);

  // config detail: {'bit': 21, 'is_signed_var': False, 'len': 2, 'name': 'sweep_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_sweep_mode_ctrl(int sweep_mode_ctrl);

  // config detail: {'bit': 19, 'is_signed_var': False, 'len': 1, 'name': 'fan_speed_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_fan_speed_mode(bool fan_speed_mode);

  // config detail: {'bit': 17, 'is_signed_var': False, 'len': 2, 'name': 'fan_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fan_mode_ctrl(int fan_mode_ctrl);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 2, 'name': 'sweep_plate_up_down', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_sweep_plate_up_down(int sweep_plate_up_down);

  // config detail: {'bit': 13, 'is_signed_var': False, 'len': 2, 'name': 'mouthpiece_up_down_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_mouthpiece_up_down_ctrl(int mouthpiece_up_down_ctrl);

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'shaker_duster_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_shaker_duster_ctrl(bool shaker_duster_ctrl);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'dedusting_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dedusting_ctrl(bool dedusting_ctrl);

  // config detail: {'bit': 3, 'is_signed_var': False, 'len': 2, 'name': 'auto_garbage_dump_start_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_auto_garbage_dump_start_ctrl(int auto_garbage_dump_start_ctrl);

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 2, 'name': 'auto_cleaning_start_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_auto_cleaning_start_ctrl(int auto_cleaning_start_ctrl);

private:
	uint8_t data[8];
};



