std_msgs/Header header 
# 转向使能状态 [] [0|1]
int8 vcu_chassis_steer_en_sta 
# 转向控制越界提醒 [] [0|1]
int8 vcu_chassis_steer_slopover 
# 转向线控模式反馈 [] [0|3]
int8 vcu_chassis_steer_work_mode 
# 转向模式反馈 [] [0|15]
int8 vcu_chassis_steer_mode_fb 
# 前转向方向盘转角反馈 [deg] [-500|500]
int32 vcu_chassis_steer_angle_fb 
# 后转向方向盘转角反馈 [deg] [-500|500]
int32 vcu_chassis_steer_angle_rear_fb 
# 设置的转向转角速度反馈 [deg/s] [0|500]
float32 vcu_chassis_steer_angle_speed_fb 
