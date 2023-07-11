# constants for chassis_rr_eds_error_code2 chassis_rf_eds_error_code2 chassis_lr_eds_error_code2 chassis_lf_eds_error_code2 
uint8 MCU_CHIP_ERROR_1=1 
uint8 PRE_CHARGE_ERROR_1=2 
uint8 MOSTFET_FAULT_1=4 

# constants for chassis_rr_eds_error_code chassis_rf_eds_error_code chassis_lr_eds_error_code chassis_lf_eds_error_code 
uint8 MCU_EEPROM_WRITE_ERROR_2=64 
uint8 HALL_SENSOR_ERROR_2=4 
uint8 OVERHEAT_2=128 
uint8 UNDER_VOLTAGE_2=2 
uint8 MOTOR_BLOCK_2=16 
uint8 SPEED_FEEDBACK_2=32 
uint8 OVER_VOLTAGE_2=8 
uint8 OVER_CURRENT_2=1 


# variables 
std_msgs/Header header 
int8 chassis_rr_eds_error_code2 	# 右后电控故障2 [] [0|255]
int8 chassis_rf_eds_error_code2 	# 右前电控故障2 [] [0|255]
int8 chassis_lr_eds_error_code2 	# 左后电控故障2 [] [0|255]
int8 chassis_lf_eds_error_code2 	# 左前电控故障2 [] [0|255]
int8 chassis_rr_eds_error_code 	# 右后电控故障 [] [0|255]
int8 chassis_rf_eds_error_code 	# 右前电控故障 [] [0|255]
int8 chassis_lr_eds_error_code 	# 左后电控故障 [] [0|255]
int8 chassis_lf_eds_error_code 	# 左前电控故障 [] [0|255]
