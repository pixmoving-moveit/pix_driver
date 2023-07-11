# constants for chassis_ebs_error_code3 
uint8 HIGH_POWER_SUPPLY_VOLTAGE_FAULT_1=2 
uint8 LOW_POWER_SUPPLY_VOLTAGE_FAULT_1=1 
uint8 EHB_MOTOR_POSITION_FAILURE_1=16 
uint8 POWER_SWITCH_FAULT_1=4 
uint8 CURRENT_SENSOR_MALFUNCTION_1=0 
uint8 PRE_DRIVE_FAULT_1=8 

# constants for chassis_back_eps_error_code chassis_front_eps_error_code 
uint8 BATTERY_OVERVOLTAGE_2=2 
uint8 OVERTEMPERATURE_2=4 
uint8 CURRENT_TRACKING_2=32 
uint8 OVERCURRENT_2=8 
uint8 OTHER_FAULTS_2=128 
uint8 CURRENT_SENSOR_2=64 
uint8 EEPROM_COMMUNICATION_2=16 
uint8 BATTERY_UNDERVOLTAGE_2=1 

# constants for chassis_ebs_error_code2 
uint8 ABNORMAL_HIGH_POWER_SUPPLY_VOLTAGE_3=2 
uint8 SEVERE_OSCILLATION_IN_EHB_HYDRAULIC_PRESSURE_3=8 
uint8 ABNORMAL_LOW_POWER_SUPPLY_VOLTAGE_3=1 
uint8 SEVERE_ABNORMALITY_IN_EHB_HYDRAULIC_PRESSURE_TRACKING_3=4 
uint8 CAN_COMMUNICATION_FAULT_3=32 
uint8 LOAD_MISMATCH_FAULT_3=16 
uint8 EHB_OVERCURRENT_FAULT_3=128 
uint8 NOT_USED_3=64 

# constants for chassis_ebs_error_code1 
uint8 CONTROLLER_TEMPERATURE_TOO_HIGH_4=128 
uint8 SLIGHT_DEVIATION_IN_EHB_HYDRAULIC_PRESSURE_TRACKING_4=2 
uint8 ABNORMAL_BRAKE_COMMAND_4=1 
uint8 HYDRAULIC_PRESSURE_SENSOR_MALFUNCTION_4=64 
uint8 COMPLETE_FAILURE_OF_TEMPERATURE_SENSOR_4=32 
uint8 SLIGHT_OSCILLATION_IN_EHB_HYDRAULIC_PRESSURE_4=4 
uint8 NOT_USED_4=16 
uint8 PARTIAL_TEMPERATURE_SENSOR_MALFUNCTION_4=8 

# constants for chassis_ebs_errorlv 
uint8 LEVEL_3_FAULT_5=3 
uint8 LEVEL_2_FAULT_5=2 
uint8 LEVEL_4_FAULT_5=4 
uint8 LEVEL_1_FAULT_5=1 
uint8 NO_FAULT_5=0 


# variables 
std_msgs/Header header 
int8 chassis_ebs_error_code3 	# 制动故障码3 [] [0|255]
int32 chassis_bms_error_code 	#  [] [0|255]
int8 chassis_back_eps_error_code 	# 后转向故障码 [] [0|255]
int8 chassis_front_eps_error_code 	# 前转向故障码 [] [0|255]
int8 chassis_ebs_error_code2 	# 制动故障码2 [] [0|255]
int8 chassis_ebs_error_code1 	# 制动故障码1 [] [0|255]
int8 chassis_ebs_errorlv 	# 制动故障级别 [] [0|15]
