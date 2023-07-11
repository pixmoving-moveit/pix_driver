# constants for dirve_en_state 
uint8 STANDBY_1=3 
uint8 TAKEOVER_1=2 
uint8 MANUAL_1=0 
uint8 AUTO_1=1 


# variables 
std_msgs/Header header 
float32 dirve_throttle_pedal_actual 	# 行驶油门实际值 [%] [0|100]
int32 dirve_flt2 	# 行驶故障码2 [] [0|255]
int32 dirve_flt1 	# 行驶故障码1 [] [0|255]
int8 dirve_en_state 	# 行驶使能状态 [] [0|3]
