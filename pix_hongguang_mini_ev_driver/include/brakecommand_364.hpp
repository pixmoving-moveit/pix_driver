#pragma once
#include "Byte.hpp"

class Brakecommand364 {
public:
	static  int32_t ID;

	Brakecommand364();

  	void UpdateData(int vcu_sts_, int vcu_selfstudy_, int vcu_keyst_, bool vcu_extbrkpressure_v_, int vcu_extbrkpressure_, int vcu_ebs_stmodereq_, int vcu_ebs_ctroll_, bool vcu_drvmode_);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 29, 'is_signed_var': False, 'len': 2, 'name': 'VCU_sts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_vcu_sts(int vcu_sts);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'VCU_SelfStudy', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_vcu_selfstudy(int vcu_selfstudy);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 2, 'name': 'VCU_KeySt', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_vcu_keyst(int vcu_keyst);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'VCU_ExtBrkpressure_V', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_vcu_extbrkpressure_v(bool vcu_extbrkpressure_v);

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'VCU_ExtBrkpressure', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|125]', 'physical_unit': 'Mpa', 'precision': 1.0, 'type': 'int'}
  void set_p_vcu_extbrkpressure(int vcu_extbrkpressure);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_StModeReq', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_vcu_ebs_stmodereq(int vcu_ebs_stmodereq);

  // config detail: {'bit': 35, 'is_signed_var': False, 'len': 4, 'name': 'VCU_EBS_ctRoll', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|14]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_vcu_ebs_ctroll(int vcu_ebs_ctroll);

  // config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name': 'VCU_DrvMode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_vcu_drvmode(bool vcu_drvmode);

private:
	uint8_t data[8];
};



