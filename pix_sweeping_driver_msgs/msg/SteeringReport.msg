# constants for steer_en_state 
uint8 STANDBY_1=3 
uint8 TAKEOVER_1=2 
uint8 MANUAL_1=0 
uint8 AUTO_1=1 


# variables 
std_msgs/Header header 
int32 steer_angle_speed_actual 	# 转向实际角速度 [deg/s] [0|250]
int32 steer_angle_rear_actual 	# 后转向实际角度 [deg] [-500|500]
int32 steer_angle_actual 	# 转向实际角度 [deg] [-500|500]
int32 steer_flt2 	# 转向故障码2 [] [0|0]
int32 steer_flt1 	# 转向故障码1 [] [0|0]
int8 steer_en_state 	# 转向使能状态 [] [0|3]
