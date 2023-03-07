#pragma once
#include <pix_hooke_driver/Byte.hpp>

class A2vwheelctrl135 {
public:
	static  int32_t ID;

	A2vwheelctrl135();

  	void UpdateData(double acu_motortorquelfctrl_, double acu_motortorquerfctrl_, double acu_motortorquelrctrl_, double acu_motortorquerrctrl_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'description': '左前电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueLfCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_acu_motortorquelfctrl(double acu_motortorquelfctrl);

  // config detail: {'bit': 16, 'description': '右前电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueRfCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_acu_motortorquerfctrl(double acu_motortorquerfctrl);

  // config detail: {'bit': 32, 'description': '左后电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueLrCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_acu_motortorquelrctrl(double acu_motortorquelrctrl);

  // config detail: {'bit': 48, 'description': '右后电机扭矩', 'is_signed_var': True, 'len': 16, 'name': 'ACU_MotorTorqueRrCtrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-200|200]', 'physical_unit': 'Nm', 'precision': 0.1, 'type': 'double'}
  void set_p_acu_motortorquerrctrl(double acu_motortorquerrctrl);

private:
	uint8_t data[8];
};



