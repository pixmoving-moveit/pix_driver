std_msgs/Header header 
# 驾驶模式反馈 [] [0|3]
int8 vcu_driving_mode_fb 
# 车辆上电状态反馈 [] [0|3]
int8 vcu_chassis_power_sta_fb 
# DC工作状态 [] [0|3]
int8 vcu_chassis_power_dc_sta 
# 车辆限速状态 [] [0|1]
int8 vcu_chassis_speed_limited_mode_fb 
# 功率限制状态 [] [0|1]
int8 vcu_chassis_power_limite_sta 
# 节能模式反馈 [] [0|3]
int8 vcu_sys_eco_mode 
# 车辆限速值反馈 [m/s] [0|50]
float32 vcu_chassis_speed_limited_val_fb 
# 低压蓄电池电压 [V] [0|25]
float32 vcu_chassis_low_power_volt_sta 
# 紧急停车状态反馈 [] [0|15]
int8 vcu_chassis_e_stop_sta_fb 
# 车辆前碰撞传感器反馈 [] [0|1]
int8 vcu_crash_front_sta 
# 车辆后碰撞传感器反馈 [] [0|1]
int8 vcu_crash_rear_sta 
# 车辆左碰撞传感器反馈 [] [0|1]
int8 vcu_crash_left_sta 
# 车辆右碰撞传感器反馈 [] [0|1]
int8 vcu_crash_right_sta 
# vcu循环计数 [] [0|15]
int32 vcu_life 
# 校验sum=byte0 xor byte1 xor...byte6 [] [0|255]
int32 vcu_check_sum 
