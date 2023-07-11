# constants for brake_en_state 
uint8 STANDBY_1=3 
uint8 TAKEOVER_1=2 
uint8 MANUAL_1=0 
uint8 AUTO_1=1 


# variables 
std_msgs/Header header 
float32 brake_pedal_actual 	# 制动踏板实际值 [%] [0|100]
int32 brake_flt2 	# 制动故障码2 [] [0|255]
int32 brake_flt1 	# 制动故障码1 [] [0|255]
int8 brake_en_state 	# 制动使能状态 [] [0|3]
