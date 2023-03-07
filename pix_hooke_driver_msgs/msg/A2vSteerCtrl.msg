std_msgs/Header header 
# 车辆转向控制使能 [] [0|1]
int8 acu_chassis_steer_en_ctrl 
# 转向模式控制 [] [0|15]
int8 acu_chassis_steer_mode_ctrl 
# 车辆转向控制（前） [deg] [-500|500]
int32 acu_chassis_steer_angle_target 
# 车辆转向控制（后） [deg] [-500|500]
int32 acu_chassis_steer_angle_rear_target 
# 车辆方向盘转角速度控制 [deg/s] [0|500]
float32 acu_chassis_steer_angle_speed_ctrl 
# 校验sum=byte0 xor byte1 xor...byte6 [] [0|255]
int32 acu_check_sum_132
