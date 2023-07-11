# constants for gear_target 
uint8 PARK_1=1 
uint8 REVERSE_1=2 
uint8 INVALID_1=0 
uint8 NEUTRAL_1=3 
uint8 DRIVE_1=4 


# variables 
std_msgs/Header header 
int32 check_sum103 	# 校验和 [] [0|255]
int8 gear_target 	# 目标挡位 [] [0|7]
bool gear_en_ctrl 	# 挡位使能 [] [0|1]
