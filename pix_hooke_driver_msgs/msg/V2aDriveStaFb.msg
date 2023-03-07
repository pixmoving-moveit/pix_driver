std_msgs/Header header 
# 驱动使能状态 [] [0|1]
int8 vcu_chassis_driver_en_sta 
# 驱动控制越界提醒 [] [0|1]
int8 vcu_chassis_diver_slopover 
# 驱动模式反馈 [] [0|3]
int8 vcu_chassis_driver_mode_sta 
# 档位状态 [] [0|3]
int8 vcu_chassis_gear_fb 
# 车速实际反馈 [m/s] [-50|50]
float32 vcu_chassis_speed_fb 
# 油门请求值反馈 [%] [0|100]
float32 vcu_chassis_throttle_padl_fb 
# 车辆加速度 [m/s2] [-20|20]
float32 vcu_chassis_accceleration_fb 
