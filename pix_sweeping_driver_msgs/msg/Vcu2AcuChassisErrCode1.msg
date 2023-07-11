# constants for bms_communication_fault chassis_emergency_sta leadacid_battery_low controler_communication_fault chassis_crash_sta eps_rear_communication_fault eps_front_communication_fault eds_rr_communication_fault eds_rf_communication_fault eds_lr_communication_fault eds_lf_communication_fault ebs_communication_fault chassis_vcu_other_err 
uint8 FALSE_1=1 
uint8 TRUE_1=0 

# constants for bms_fault1 bms_fault2 bms_fault3 bms_fault4 bms_fault5 bms_fault6 bms_fault7 bms_fault8 bms_fault9 bms_fault10 bms_fault11 bms_fault12 bms_fault13 
uint8 NORMAL_2=0 
uint8 LEVEL_1_ALARM_2=1 
uint8 LEVEL_2_ALARM_2=2 
uint8 LEVEL_3_ALARM_2=3 


# variables 
std_msgs/Header header 
int8 bms_communication_fault 	# BMS通讯故 [] [0|1]
int8 chassis_emergency_sta 	# 底盘急停 [] [0|1]
int8 leadacid_battery_low 	# 铅酸电池电量低 [] [0|1]
int8 controler_communication_fault 	# 控制器通讯故障 [] [0|1]
int8 chassis_crash_sta 	# 防撞条触发状态 [] [0|1]
int8 eps_rear_communication_fault 	# 后EPS通讯故障 [] [0|1]
int8 eps_front_communication_fault 	# 前EPS通讯故障 [] [0|1]
int8 eds_rr_communication_fault 	# 右后电控通讯故障 [] [0|1]
int8 eds_rf_communication_fault 	# 右前电控通讯故障 [] [0|1]
int8 eds_lr_communication_fault 	# 左后电控通讯故障 [] [0|1]
int8 eds_lf_communication_fault 	# 左前电控通讯故障 [] [0|1]
int8 ebs_communication_fault 	# EHB通讯故障 [] [0|1]
int8 chassis_vcu_other_err 	# VCU其他故障 [] [0|255]
int8 bms_fault1 	# 单体过压故障 [] [0|17]
int8 bms_fault2 	# 单体低压故障 [] [0|17]
int8 bms_fault3 	# 单体压差过大故障 [] [0|17]
int8 bms_fault4 	# 放电高温故障 [] [0|17]
int8 bms_fault5 	# 放电低温故障 [] [0|17]
int8 bms_fault6 	# 充电高温故障 [] [0|17]
int8 bms_fault7 	# 充电低温故障 [] [0|17]
int8 bms_fault8 	# 单体温差过大故障 [] [0|17]
int8 bms_fault9 	# 总压过低故障 [] [0|17]
int8 bms_fault10 	# 充电过流故障 [] [0|17]
int8 bms_fault11 	# 放电过流故障 [] [0|17]
int8 bms_fault12 	# SOC过低故障 [] [0|17]
int8 bms_fault13 	# 总压过低故障 [] [0|17]
