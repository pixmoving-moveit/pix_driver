# constants for mowing_speed_ctrl sweep_mode_ctrl fan_mode_ctrl 
uint8 LOW_1=1 
uint8 OFF_1=0 
uint8 MID_1=2 
uint8 HIGH_1=3 

# constants for fan_speed_mode 
uint8 DEFAULT_2=0 
uint8 SPEED_2=1 

# constants for sweep_plate_up_down mouthpiece_up_down_ctrl 
uint8 NO_3=0 
uint8 DOWN_3=2 
uint8 UP_3=1 

# constants for auto_garbage_dump_start_ctrl auto_cleaning_start_ctrl 
uint8 NO_4=0 
uint8 STOP_4=2 
uint8 START_4=1 


# variables 
std_msgs/Header header 
int32 fan_speed_ctrl 	# 风机速度控制 [rpm] [0|65535]
int8 mowing_speed_ctrl 	# 割草转速挡位控制 [] [0|3]
int8 sweep_mode_ctrl 	# 扫盘转速挡位控制 [] [0|3]
int8 fan_speed_mode 	# 风机速度模式 [] [0|1]
int8 fan_mode_ctrl 	# 风机工作模式 [] [0|3]
int8 sweep_plate_up_down 	# 扫盘升降 [] [0|3]
int8 mouthpiece_up_down_ctrl 	# 吸盘升降控制 [] [0|3]
bool shaker_duster_ctrl 	# 振尘控制 [] [0|1]
bool dedusting_ctrl 	# 喷雾降尘控制 [] [0|1]
int8 auto_garbage_dump_start_ctrl 	# 一键倾倒控制 [] [0|3]
int8 auto_cleaning_start_ctrl 	# 一键清扫控制 [] [0|3]
bool charge_alignment_state_feedback # 充电桩是否对准状态反馈 [0|1]
 