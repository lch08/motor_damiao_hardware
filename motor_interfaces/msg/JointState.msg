#每个关节独立的错误消息

uint32 ERR_CODE_ENABLE          = 0x00  # 使能
uint32 ERR_CODE_DISABLE         = 0x01  # 失能
uint32 ERR_CODE_DISCONNECT      = 0x02  # 断开连接
uint32 ERR_CODE_OVER_LOAD       = 0x03  # 过载
uint32 ERR_CODE_OVER_CURRENT    = 0x04  # 过流
uint32 ERR_CODE_OVER_VOLTAGE    = 0x05  # 过电压
uint32 ERR_CODE_LOW_VOLTAGE     = 0x06  # 低电压
uint32 ERR_CODE_MOS_OVER_TEMP   = 0x07  # mos超温
uint32 ERR_CODE_MOTOR_OVER_TEMP = 0x08  # 电机超温


string  joint_name  # 关节名
uint32  err_code    # 错误代码
float32 mos_temp    # mos温度
float32 motor_temp  # 电机温度