#pragma once
#include <pix_sweeping_driver/Byte.hpp>

class AcuSweepCtrlCmd {
public:
	static  int32_t ID;

	AcuSweepCtrlCmd();

  	void UpdateData(int fan_speed_ctrl, int mowing_speed_ctrl, int sweep_mode_ctrl, int fan_speed_mode, int fan_mode_ctrl, int sweep_plate_up_down, int mouthpiece_up_down_ctrl, bool shaker_duster_ctrl, bool dedusting_ctrl, int auto_garbage_dump_start_ctrl, int auto_cleaning_start_ctrl);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 47, 'description': '风机速度控制', 'is_signed_var': True, 'len': 16, 'name': 'fan_speed_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
  void set_p_fan_speed_ctrl(int fan_speed_ctrl);

  // config detail: {'bit': 25, 'description': '割草转速挡位控制', 'enum': {0: 'OFF', 1: 'LOW', 2: 'MID', 3: 'HIGH'}, 'is_signed_var': False, 'len': 2, 'name': 'mowing_speed_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_mowing_speed_ctrl(int mowing_speed_ctrl);

  // config detail: {'bit': 21, 'description': '扫盘转速挡位控制', 'enum': {0: 'OFF', 1: 'LOW', 2: 'MID', 3: 'HIGH'}, 'is_signed_var': False, 'len': 2, 'name': 'sweep_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_sweep_mode_ctrl(int sweep_mode_ctrl);

  // config detail: {'bit': 19, 'description': '风机速度模式', 'enum': {0: 'DEFAULT', 1: 'SPEED'}, 'is_signed_var': False, 'len': 1, 'name': 'fan_speed_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_fan_speed_mode(int fan_speed_mode);

  // config detail: {'bit': 17, 'description': '风机工作模式', 'enum': {0: 'OFF', 1: 'LOW', 2: 'MID', 3: 'HIGH'}, 'is_signed_var': False, 'len': 2, 'name': 'fan_mode_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_fan_mode_ctrl(int fan_mode_ctrl);

  // config detail: {'bit': 15, 'description': '扫盘升降', 'enum': {0: 'NO', 1: 'UP', 2: 'DOWN'}, 'is_signed_var': False, 'len': 2, 'name': 'sweep_plate_up_down', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_sweep_plate_up_down(int sweep_plate_up_down);

  // config detail: {'bit': 13, 'description': '吸盘升降控制', 'enum': {0: 'NO', 1: 'UP', 2: 'DOWN'}, 'is_signed_var': False, 'len': 2, 'name': 'mouthpiece_up_down_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_mouthpiece_up_down_ctrl(int mouthpiece_up_down_ctrl);

  // config detail: {'bit': 9, 'description': '振尘控制', 'is_signed_var': False, 'len': 1, 'name': 'shaker_duster_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_shaker_duster_ctrl(bool shaker_duster_ctrl);

  // config detail: {'bit': 8, 'description': '喷雾降尘控制', 'is_signed_var': False, 'len': 1, 'name': 'dedusting_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dedusting_ctrl(bool dedusting_ctrl);

  // config detail: {'bit': 3, 'description': '一键倾倒控制', 'enum': {0: 'NO', 1: 'START', 2: 'STOP'}, 'is_signed_var': False, 'len': 2, 'name': 'auto_garbage_dump_start_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_auto_garbage_dump_start_ctrl(int auto_garbage_dump_start_ctrl);

  // config detail: {'bit': 1, 'description': '一键清扫控制', 'enum': {0: 'NO', 1: 'START', 2: 'STOP'}, 'is_signed_var': False, 'len': 2, 'name': 'auto_cleaning_start_ctrl', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_auto_cleaning_start_ctrl(int auto_cleaning_start_ctrl);

private:
	uint8_t data[8];
};



