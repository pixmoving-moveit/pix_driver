# constants for gear_actual 
uint8 PARK_1=1 
uint8 REVERSE_1=2 
uint8 INVALID_1=0 
uint8 NEUTRAL_1=3 
uint8 DRIVE_1=4 


# variables 
std_msgs/Header header 
int32 gear_flt 	# 挡位故障 [] [0|0]
int8 gear_actual 	# 实际挡位 [] [0|7]
