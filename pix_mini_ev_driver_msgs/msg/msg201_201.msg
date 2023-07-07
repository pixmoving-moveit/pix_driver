Header header 
# 实际的加速踏板位置：有效范围0～100%。 [%] [0|100]
float32 AccPedAct 
# 加速踏板位置指令回传：有效范围0～100%。 [-] [0|255]
int32 AccPedCmd 
# 执行的加速踏板位置：有效范围0～100%。 [%] [0|100]
float32 AccPedExe 
# 驱动控制状态：0=禁用，1=使能，2=故障，3保留。 [-] [0|3]
int32 AccCtrlStat 
# 报文计数器。 [-] [0|15]
int32 AccStatCnt 
# 报文校验和。 [-] [0|255]
int32 AccStatCks 
