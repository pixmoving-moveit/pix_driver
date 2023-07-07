Header header 
# 加速踏板位置指令，有效范围0～100%。 [%] [0|100]
float32 AccPedCmd 
# 加速踏板控制使能：0=禁用，1=使能。 [-] [0|1]
bool AccCtrlEna 
# 加速踏板位置校验值：有效范围0～100%。 [%] [0|100]
float32 AccPedInv 
# 报文计数器。 [-] [0|15]
int32 AccCtrlCnt 
# 报文校验和。 [-] [0|255]
int32 AccCtrlCks 
# - [-] [0|1]
bool AccTkoDis 
