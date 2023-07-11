
# variables 
std_msgs/Header header 
int32 scu_mowing_speed_fb 	# 割草电机实际转速 [rpm] [0|65535]
int32 push_rod_travel_difference_alarm 	# 推杆行程差值报警 [] [0|3]
bool end_of_garbage_dumping_des 	# 一键垃圾倾倒下降结束 [] [0|1]
bool end_of_garbage_dumping 	# 垃圾倾倒倾翻结束 [] [0|1]
int32 scu_fan_speed_fb 	# 风机电机实际转速 [rpm] [0|65535]
int32 scu_liquid_leve 	# 清水箱液位值 [] [0|255]
bool scu_heartbeat 	# 上装控制器心跳 [] [0|1]
bool scan_controller_communication_fault 	# 扫盘控制器通信故障报警 [] [0|1]
bool fan_controller_communication_fault 	# 风机控制器通信故障报警 [] [0|1]
bool scu_filter_clogging 	# 滤芯堵赛报警 [] [0|1]
bool scu_sweep_life_end 	# 扫盘磨损结束报警 [] [0|1]
bool scu_dustbin_full 	# 垃圾箱满溢报警 [] [0|1]
bool scu_liquid_leve_high 	# 清水箱高液位报警 [] [0|1]
bool scu_liquid_level_low 	# 清水箱低液位报警 [] [0|1]
