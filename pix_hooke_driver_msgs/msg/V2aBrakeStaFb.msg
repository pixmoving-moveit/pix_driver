std_msgs/Header header 
# 制动使能状态 [] [0|1]
int8 vcu_chassis_brake_en_sta 
# 制动灯状态反馈 [] [0|1]
int8 vcu_vehicle_brake_lamp_fb 
# 驻车状态 [] [0|3]
int8 vcu_chassis_epb_fb 
# 制动实际反馈 [%] [0|100]
float32 vcu_chassis_brake_padl_fb 
# AEB开启状态反馈 [] [0|1]
int8 vcu_aeb_en_sta_fb 
# AEB触发状态反馈 [] [0|1]
int8 vcu_aeb_trigger_sta_fb 
