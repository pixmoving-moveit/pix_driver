# constants for turn_light_ctrl 
uint8 TURNLAMPOFF_1=0 
uint8 HAZARDWARNINGLAMPSTSON_1=3 
uint8 LEFTTURNLAMPON_1=1 
uint8 RIGHTTURNLAMPON_1=2 

# constants for drive_mode_ctrl 
uint8 SPEED_DRIVE_2=1 
uint8 THROTTLE_PADDLE_DRIVE_2=0 

# constants for steer_mode_ctrl 
uint8 SYNC_DIRECTION_STEER_3=2 
uint8 NON_DIRECTION_STEER_3=1 
uint8 STANDARD_STEER_3=0 


# variables 
std_msgs/Header header 
int32 check_sum_105 	# 校验和 [] [0|255]
bool headlight_ctrl 	# 大灯控制 [] [0|1]
int8 turn_light_ctrl 	# 转向灯控制 [] [0|3]
int8 drive_mode_ctrl 	# 驾驶模式控制 [] [0|7]
int8 steer_mode_ctrl 	# 转向模式控制 [] [0|7]
