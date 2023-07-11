# constants for parking_actual 
uint8 RELEASE_1=0 
uint8 PARKINGTRIGGER_1=1 


# variables 
std_msgs/Header header 
int32 park_flt 	# 驻车故障 [] [0|0]
int8 parking_actual 	# 实际驻车状态 [] [0|1]
