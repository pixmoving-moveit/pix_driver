# constants for vcu_sweep_plate_up_down_sta_fb 
uint8 NO_1=0 
uint8 DOWN_1=2 
uint8 UP_1=1 


# variables 
std_msgs/Header header 
bool sweep_emergency_sig_fb 	# 清扫系统紧急停止反馈 [] [0|1]
int8 vcu_sweep_plate_up_down_sta_fb 	# 扫盘升降状态反馈 [] [0|3]
bool vcu_auto_garbage_dump_sta_fb 	# 一键倾倒状态反馈 [] [0|1]
bool vcu_auto_cleaning_sta_fb 	# 一键清扫状态反馈 [] [0|1]
