Header header 
# 雨刮控制指令:0=不开雨刮,1=雨刮低速挡,2=雨刮高速挡,3=雨刮喷水挡。 [] [0|3]
int32 WiperCmd 
# 鸣笛控制指令:0:不鸣笛,1:鸣笛。 [-] [0|1]
bool HornCmd 
# 档位控制指令:2=R档,3=N档,4=D档,其他保留。 [-] [0|255]
int32 GearCmd 
# 档位控制使能:0=禁用,1=使能。 [-] [0|1]
bool GearCtrlEna 
# 转向灯控制指令:0=不控制转向灯,1=左转向灯闪烁,2=右转向灯闪烁,3保留。 [-] [0|3]
int32 TurnLightCmd 
# 远近光灯控制指令:0=不开灯,1=开灯,其他:保留。 [-] [0|3]
int32 BeamLightCmd 
# 报文计数器。 [-] [0|15]
int32 GearCtrlCnt 
# 报文校验和。 [-] [0|255]
int32 GearCtrlCks 
