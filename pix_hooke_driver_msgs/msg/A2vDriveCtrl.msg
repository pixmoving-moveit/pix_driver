std_msgs/Header header 
# 车辆加速控制使能 [] [0|1]
int8 acu_chassis_driver_en_ctrl 
# 驱动模式控制 [] [0|3]
int8 acu_chassis_driver_mode_ctrl 
# 档位控制 [] [0|15]
int8 acu_chassis_gear_ctrl 
# 车辆速度控制 [m/s] [0|50]
float32 acu_chassis_speed_ctrl 
# 车辆油门控制 [%] [0|100]
float32 acu_chassis_throttle_pdl_target 
# 循环计数0~15 [] [0|15]
int32 acu_drive_life_sig 
# 校验sum=byte0 xor byte1 xor...byte6 [] [0|255]
int32 acu_check_sum_130 
