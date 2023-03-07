std_msgs/Header header 
# 预留 [] [0|15]
int32 vcu_chassis_bms_reserved_1 
# 车辆充电状态 [] [0|3]
int8 vcu_chassis_power_charge_sta 
# 充电枪连接状态 [] [0|1]
int8 vcu_chassis_power_charge_sock_sta 
# 车辆动力电池电量 [%] [0|100]
int32 vcu_chassis_power_soc_fb 
# 车辆动力电池电压 [V] [0|1000]
float32 vcu_chassis_power_volt_fb 
# 车辆动力电池电流 [A] [-1000|1000]
float32 vcu_chassis_power_curr_fb 
# BMS最高单体温度 [℃] [-40|80]
int32 vcu_chassis_bms_max_temp 
# 预留 [] [0|255]
int32 vcu_chassis_bms_reserved_2 
