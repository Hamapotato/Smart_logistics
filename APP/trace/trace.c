#include "trace.h"

int16_t Off_Value     = 0;
int16_t Omega_Value   = 0;
int16_t Target_Sensor = 24;

uint8_t reverse(uint8_t x)
{
    x = (((x & 0xaa) >> 1) | ((x & 0x55) << 1));
    x = (((x & 0xcc) >> 2) | ((x & 0x33) << 2));

    return ((x >> 4) | (x << 4));
}

void FRONT_BACK_TRACING(int8_t speed, u8 mode)
{
    uint8_t OFF_JUDGE;
    if (mode == 1) {
        OFF_JUDGE   = reverse(~F_GW_digital_Read());
    } else if (mode == 0) {
        OFF_JUDGE   = reverse(~B_GW_digital_Read());
    }

    switch (OFF_JUDGE) {
        case 0x18: { // 00011000
            Off_Value = 0;
        } break;
        case 0x30: { // 00110000
            Off_Value = CHASSIS_OFF_VALUE_1;
        } break;
        case 0x60: { // 01100000
            Off_Value = CHASSIS_OFF_VALUE_2;
        } break;
        case 0xC0: { // 11000000
            Off_Value = CHASSIS_OFF_VALUE_3;
        } break;
        case 0x80: { // 10000000
            Off_Value = CHASSIS_OFF_VALUE_4;
        } break;
        case 0x0C: { // 00001100
            Off_Value = -CHASSIS_OFF_VALUE_1;
        } break;
        case 0x06: { // 00000110
            Off_Value = -CHASSIS_OFF_VALUE_2;
        } break;
        case 0x03: { // 00000011
            Off_Value = -CHASSIS_OFF_VALUE_3;
        } break;
        case 0x01: { // 00000001
            Off_Value = -CHASSIS_OFF_VALUE_4;
        } break;
        case 0x38: { // 00111000
            Off_Value = CHASSIS_OFF_VALUE_2;
        } break;
        case 0x70: { // 01110000
            Off_Value = CHASSIS_OFF_VALUE_3;
        } break;
        case 0xE0: { // 11100000
            Off_Value = CHASSIS_OFF_VALUE_3;
        } break;
        case 0x1C: { // 00011100
            Off_Value = -CHASSIS_OFF_VALUE_2;
        } break;
        case 0x0E: { // 00001110
            Off_Value = -CHASSIS_OFF_VALUE_3;
        } break;
        case 0x07: { // 00000111
            Off_Value = -CHASSIS_OFF_VALUE_3;
        } break;
        default: {
            Off_Value = 0;
        } break;
    }
    //
    // 偏移控制
    // switch (OMEGA_JUDGE) {
    //    case 0x18: { // 00011000
    //        Omega_Value = 0;
    //    } break;
    //    case 0x30: { // 00110000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x60: { // 01100000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0xC0: { // 11000000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x0C: { // 00001100
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x06: { // 00000110
    //        Omega_Value = CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0x03: { // 00000011
    //        Omega_Value = CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x38: { // 00111000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x70: { // 01110000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0xE0: { // 11100000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x1C: { // 00011100
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x0E: { // 00001110
    //        Omega_Value = CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0x07: { // 00000111
    //        Omega_Value = CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x10: { // 00010000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x20: { // 00100000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x40: { // 01000000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_MID;
    //    }
    //    case 0x80: { // 10000000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_HIGH;
    //    }
    //    case 0x08: { // 00001000
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x04: { // 00000100
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x02: { // 00000010
    //        Omega_Value = CHASSIS_OMEGA_VALUE_MID;
    //    }
    //    case 0x01: { // 00000001
    //        Omega_Value = CHASSIS_OMEGA_VALUE_HIGH;
    //    }
    //    default: {
    //        Omega_Value = 0;
    //    } break;
    //}

    /* 纠偏量按“原地旋转”组合作用：右侧 201/202 一组、左侧 203/204 一组，两组反向。
       2026-10-10 用 MoveTest 实测：201=右前 202=右后 203=左后 204=左前，四个电机的
       “正转”都是车轮前进方向，驱动板认正负号。轮速空间是一组 Hadamard 基：
           前后平移 (+1,+1,+1,+1)     左右平移 (+1,-1,+1,-1)
           原地旋转 (+1,+1,-1,-1)     空转     (+1,-1,-1,+1)   <- 对角轮对顶，推不动
       下面四行里 Off_Value 的符号是 (-1,-1,+1,+1)，也就是“原地旋转”的反相——这是对的。
       （中间有一版把它误当成空转组写成了 (+1,-1,-1,+1)，那样纠偏量发下去完全不起作用。）
       纠偏方向反了（车头往远离黑线的方向扭）就把 TRACE_STEER_SIGN 改成 -1。 */
    Motor_201.Target_Speed = speed + Omega_Value - TRACE_OFF(speed);
    Motor_202.Target_Speed = speed - Omega_Value - TRACE_OFF(speed);
    Motor_203.Target_Speed = speed + Omega_Value + TRACE_OFF(speed);
    Motor_204.Target_Speed = speed - Omega_Value + TRACE_OFF(speed);

    // Motor_201.PID_Output = Incremental_PID_calc(&PID_201,Motor_201.Target_Speed,Motor_201.Real_Speed);
    // Motor_202.PID_Output = Incremental_PID_calc(&PID_202,Motor_202.Target_Speed,Motor_202.Real_Speed);
    // Motor_203.PID_Output = Incremental_PID_calc(&PID_203,Motor_203.Target_Speed,Motor_203.Real_Speed);
    // Motor_204.PID_Output = Incremental_PID_calc(&PID_204,Motor_204.Target_Speed,Motor_204.Real_Speed);

    Motor_201.PID_Output = Motor_201.Target_Speed;
    Motor_202.PID_Output = Motor_202.Target_Speed;
    Motor_203.PID_Output = Motor_203.Target_Speed;
    Motor_204.PID_Output = Motor_204.Target_Speed;

    MOTOR_CONTROL_DATA_Predit(&Motor_203, &Motor_202, &Motor_204, &Motor_201);
}

void RIGHT_LEFT_TRACING(int8_t speed, u8 mode)
{
    uint8_t OFF_JUDGE;
    if (mode == 1) {
        OFF_JUDGE   = reverse(~L_GW_digital_Read());
    } else if (mode == 0) {
        OFF_JUDGE   = reverse(~R_GW_digital_Read());
    }

    // Off_Value = Positional_PID_calc(&SENSOR,Target_Sensor,OFF_JUDGE);

    // 偏转控制
    switch (OFF_JUDGE) {
        case 0x18: { // 00011000
            Off_Value = 0;
            //switch (Omega_JUDGE) {
            //    case 0x18: { // 00011000
            //        Off_Value = 0;
            //    } break;
            //    case 0x30: { // 00110000
            //        Off_Value = -CHASSIS_OFF_VALUE_1;
            //    } break;
            //    case 0x60: { // 01100000
            //        Off_Value = -CHASSIS_OFF_VALUE_2;
            //    } break;
            //    case 0xC0: { // 11000000
            //        Off_Value = -CHASSIS_OFF_VALUE_3;
            //    } break;
            //    case 0x80: { // 10000000
            //        Off_Value = -CHASSIS_OFF_VALUE_4;
            //    } break;
            //    case 0x0C: { // 00001100
            //        Off_Value = CHASSIS_OFF_VALUE_1;
            //    } break;
            //    case 0x06: { // 00000110
            //        Off_Value = CHASSIS_OFF_VALUE_2;
            //    } break;
            //    case 0x03: { // 00000011
            //        Off_Value = CHASSIS_OFF_VALUE_3;
            //    } break;
            //    case 0x01: { // 00000001
            //        Off_Value = CHASSIS_OFF_VALUE_4;
            //    } break;
            //    case 0x38: { // 00111000
            //        Off_Value = -CHASSIS_OFF_VALUE_2;
            //    } break;
            //    case 0x70: { // 01110000
            //        Off_Value = -CHASSIS_OFF_VALUE_3;
            //    } break;
            //    case 0xE0: { // 11100000
            //        Off_Value = -CHASSIS_OFF_VALUE_3;
            //    } break;
            //    case 0x1C: { // 00011100
            //        Off_Value = CHASSIS_OFF_VALUE_2;
            //    } break;
            //    case 0x0E: { // 00001110
            //        Off_Value = CHASSIS_OFF_VALUE_3;
            //    } break;
            //    case 0x07: { // 00000111
            //        Off_Value = CHASSIS_OFF_VALUE_3;
            //    } break;
            //    default: {
            //        Off_Value = 0;
            //    } break;
            //}
        } break;
        case 0x30: { // 00110000
            Off_Value = CHASSIS_OFF_VALUE_1;
        } break;
        case 0x60: { // 01100000
            Off_Value = CHASSIS_OFF_VALUE_2;
        } break;
        case 0xC0: { // 11000000
            Off_Value = CHASSIS_OFF_VALUE_3;
        } break;
        case 0x80: { // 10000000
            Off_Value = CHASSIS_OFF_VALUE_4;
        } break;
        case 0x0C: { // 00001100
            Off_Value = -CHASSIS_OFF_VALUE_1;
        } break;
        case 0x06: { // 00000110
            Off_Value = -CHASSIS_OFF_VALUE_2;
        } break;
        case 0x03: { // 00000011
            Off_Value = -CHASSIS_OFF_VALUE_3;
        } break;
        case 0x01: { // 00000001
            Off_Value = -CHASSIS_OFF_VALUE_4;
        } break;
        case 0x38: { // 00111000
            Off_Value = CHASSIS_OFF_VALUE_2;
        } break;
        case 0x70: { // 01110000
            Off_Value = CHASSIS_OFF_VALUE_3;
        } break;
        case 0xE0: { // 11100000
            Off_Value = CHASSIS_OFF_VALUE_3;
        } break;
        case 0x1C: { // 00011100
            Off_Value = -CHASSIS_OFF_VALUE_2;
        } break;
        case 0x0E: { // 00001110
            Off_Value = -CHASSIS_OFF_VALUE_3;
        } break;
        case 0x07: { // 00000111
            Off_Value = -CHASSIS_OFF_VALUE_3;
        } break;
        default: {
            Off_Value = 0;
        } break;
    }
    // 偏移控制
    // switch (OMEGA_JUDGE) {
    //    case 0x18: { // 00011000
    //        Omega_Value = 0;
    //    } break;
    //    case 0x30: { // 00110000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x60: { // 01100000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0xC0: { // 11000000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x0C: { // 00001100
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x06: { // 00000110
    //        Omega_Value = CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0x03: { // 00000011
    //        Omega_Value = CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x38: { // 00111000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x70: { // 01110000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0xE0: { // 11100000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x1C: { // 00011100
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    } break;
    //    case 0x0E: { // 00001110
    //        Omega_Value = CHASSIS_OMEGA_VALUE_MID;
    //    } break;
    //    case 0x07: { // 00000111
    //        Omega_Value = CHASSIS_OMEGA_VALUE_HIGH;
    //    } break;
    //    case 0x10: { // 00010000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x20: { // 00100000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x40: { // 01000000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_MID;
    //    }
    //    case 0x80: { // 10000000
    //        Omega_Value = -CHASSIS_OMEGA_VALUE_HIGH;
    //    }
    //    case 0x08: { // 00001000
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x04: { // 00000100
    //        Omega_Value = CHASSIS_OMEGA_VALUE_LOW;
    //    }
    //    case 0x02: { // 00000010
    //        Omega_Value = CHASSIS_OMEGA_VALUE_MID;
    //    }
    //    case 0x01: { // 00000001
    //        Omega_Value = CHASSIS_OMEGA_VALUE_HIGH;
    //    }
    //    default: {
    //        Omega_Value = 0;
    //    } break;
    //}

    /* 同 FRONT_BACK_TRACING：纠偏量按“原地旋转”组合作用，Off_Value 符号 (-1,-1,+1,+1)。 */
    Motor_201.Target_Speed = Omega_Value + speed - TRACE_OFF(speed);
    Motor_202.Target_Speed = Omega_Value - speed - TRACE_OFF(speed);
    Motor_203.Target_Speed = Omega_Value + speed + TRACE_OFF(speed);
    Motor_204.Target_Speed = Omega_Value - speed + TRACE_OFF(speed);

    // Motor_201.PID_Output = Incremental_PID_calc(&PID_201,Motor_201.Target_Speed,Motor_201.Real_Speed);
    // Motor_202.PID_Output = Incremental_PID_calc(&PID_202,Motor_202.Target_Speed,Motor_202.Real_Speed);
    // Motor_203.PID_Output = Incremental_PID_calc(&PID_203,Motor_203.Target_Speed,Motor_203.Real_Speed);
    // Motor_204.PID_Output = Incremental_PID_calc(&PID_204,Motor_204.Target_Speed,Motor_204.Real_Speed);

    Motor_201.PID_Output = Motor_201.Target_Speed;
    Motor_202.PID_Output = Motor_202.Target_Speed;
    Motor_203.PID_Output = Motor_203.Target_Speed;
    Motor_204.PID_Output = Motor_204.Target_Speed;

    MOTOR_CONTROL_DATA_Predit(&Motor_203, &Motor_202, &Motor_204, &Motor_201);
}

void Revolve(u8 speed)
{
    /* 原地旋转 = 右侧(201/202) 与 左侧(203/204) 反向。
       原来写的是 (+1,-1,-1,+1)，那是空转组（对角轮对顶，合力合力矩都是 0），
       发下去车根本不动。2026-10-10 实测确认，改成 (+1,+1,-1,-1)。 */
    Motor_201.Target_Speed = speed;
    Motor_202.Target_Speed = speed;
    Motor_203.Target_Speed = -speed;
    Motor_204.Target_Speed = -speed;

    /* 原来这里漏了把 Target_Speed 写进 PID_Output，而 MOTOR_CONTROL_DATA_Predit()
       读的是 PID_Output，所以“原地旋转”发出去的一直是上一次巡线留下的旧值。 */
    Motor_201.PID_Output = Motor_201.Target_Speed;
    Motor_202.PID_Output = Motor_202.Target_Speed;
    Motor_203.PID_Output = Motor_203.Target_Speed;
    Motor_204.PID_Output = Motor_204.Target_Speed;

    MOTOR_CONTROL_DATA_Predit(&Motor_203, &Motor_202, &Motor_204, &Motor_201);
}