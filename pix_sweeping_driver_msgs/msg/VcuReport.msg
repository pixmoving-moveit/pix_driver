# constants for turn_light_actual 
uint8 HAZARDWARNINGLAMPSTSON_1=3 
uint8 RIGHTTURNLAMPSTSON_1=2 
uint8 TURNLAMPSTSOFF_1=0 
uint8 LEFTTURNLAMPSTSON_1=1 

# constants for car_work_state 
uint8 WORK2_2=2 
uint8 ERROR_2=6 
uint8 INIT_2=0 
uint8 WORK_2=4 
uint8 ESTOP_2=5 
uint8 WORK1_2=1 
uint8 CRASH_2=7 
uint8 WORK3_2=3 

# constants for car_power_state 
uint8 READY_3=2 
uint8 ON_3=1 
uint8 OFF_3=0 

# constants for vehicle_mode_state 
uint8 STANDBY_MODE_4=3 
uint8 AUTO_MODE_4=1 
uint8 EMERGENCY_MODE_4=2 
uint8 MANUAL_REMOTE_MODE_4=0 

# constants for drive_mode_status 
uint8 SPEED_DRIVE_MODE_5=1 
uint8 THROTTLE_PADDLE_DRIVE_MODE_5=0 

# constants for steer_mode_status 
uint8 STANDARD_STEER_MODE_6=0 
uint8 SYNC_DIRECTION_STEER_MODE_6=2 
uint8 NON_DIRECTION_STEER_MODE_6=1 


# variables 
std_msgs/Header header 
int8 turn_light_actual 	# 转向灯实际状态 [] [0|3]
bool aeb_trigger_state 	# AEB触发状态 [] [0|1]
bool headlight_actual 	# 大灯实际状态 [] [0|1]
int8 car_work_state 	# 车辆工作状态 [] [0|7]
int8 car_power_state 	# 车辆上电状态 [] [0|1]
int32 vehicle_errcode 	# 车辆故障码 [] [0|255]
bool aeb_brake_state 	# AEB制动状态 [] [0|1]
bool front_crash_state 	# 前碰撞状态 [] [0|1]
bool back_crash_state 	# 后碰撞状态 [] [0|1]
int8 vehicle_mode_state 	# 车辆模式状态 [] [0|3]
int8 drive_mode_status 	# 行驶模式状态 [] [0|7]
float32 vehicle_speed 	# 车辆速度 [m/s] [-32.768|32.767]
int8 steer_mode_status 	# 转向模式 [] [0|7]
bool brake_light_actual 	# 制动灯实际状态 [] [0|1]
float32 vehicle_acc 	# 车辆加速度 [m/s^2] [-10|10]
