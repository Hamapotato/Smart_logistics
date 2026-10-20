#include "main.h"

/* ---------------------------------------------------------------------------
 * [红外启动] 反射式红外 TCRT5000 接在 PD12 上（代号 START_FLAG）。
 *
 * INFRAED_Init() 已把它配成上拉输入，所以空闲时引脚读数是 1。
 * sensor.h 里其它红外传感器接线相同，代码统一按“低电平有效”处理
 * （宏里都带了 !），所以这里也跟着这样定义：读到 0 就是“有信号”。
 *
 * 触发必须先待命：上电后要至少看到一次引脚为高，才允许触发。
 * 这样开机瞬间车不会自己窜出去；而且万一模块其实高电平有效，
 * 挥一次手也能启动 —— 挥手让它待命，手拿开就是触发。
 *
 * 未启动时数码管显示 PD12 的原始电平，方便肉眼验证传感器：
 *   0001 = 引脚高（空闲）    0000 = 引脚低（收到信号）
 * --------------------------------------------------------------------------- */
#define IR_DETECTED      (!START_FLAG) /* 读到信号 */
#define IR_CLEAR         (START_FLAG)  /* 没信号   */
#define IR_CONFIRM_COUNT 3             /* 连续采样次数，防抖 */
#define TUBE_STARTED     1111          /* 启动后数码管显示 */

/* ---------------------------------------------------------------------------
 * [行车方向] 2026-10-10 用 MoveTest 实测出来的底盘的说话方式：
 *     轮速组合 (+,+,+,+) 给正速度   = 车轮前进, 车往前走
 *     轮速组合 (+,-,+,-) 给正速度   = 车往【左】平移
 * 也就是「前进」要给正速度、「右移」要给负速度。下面四行按这个写。
 *
 * 万一实车还是反的（喊「前进」却往后走），把这一个数改成 -1 就行，
 * 不用动下面那八行。
 * ------------------------------------------------------------------------- */
#define DRIVE_SIGN  (+1)

int main()
{
    u8 SPEED = 10;
    u8 TFLAG = 0;
    u8 IR_ARMED = 0;
    u8 IR_COUNT = 0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    LED_EX_Init();
    USART1_Init(19200);
    MOTOR_Init();
    TUBES_Driver_Init();
    GREY_Init();
    INFRAED_Init();
    OBSTACLE_Init();
    TUBES_SetBrightness(6);
    ROUTE_CHOSE_KEY_Init();
    TEAM_CHOSE_KEY_Init();
    Steering_Init();
    TIM7_Init();
    /* Park the throw arm at its rest stop before the package is loaded.
       Needs TIM7_Init() above for its 30ms tick. */
    Steering_Home();

    ASR_IIC_Init();
    delay_ms(200);
    delay_ms(3000);
    while (1) {
        u8 result = 0;
        Steering_Task();
        result    = Asr_Result(); // 返回识别结果，即识别到的词条编号
        if (result != 0) {
            if (result == 0x01) {
                Direction = FRONT; // 前进
            } else if (result == 0x02) {
                Direction = BACK; // 后退
            } else if (result == 0x03) {
                Direction = LEFT; // 左移
            } else if (result == 0x04) {
                Direction = RIGHT; // 右移
            } else if (result == 0x09) {
                Direction = STOP; // 停止
            } else if (result == 0x1C) {
                Direction = REVOLVE; // 旋转
            } else if (result == 0x0D) {
                SPEED += 5; // 加速
            } else if (result == 0x0E) {
                if (SPEED > 5) SPEED -= 5; // 减速（SPEED 是 u8，减到 0 再减会下溢成 251）
            } else if (result == 0x66) {
                if (TEAM_CHOSE == 1) {
                    BLUE_FLAG = 1;
                    RED_FLAG  = 0;
                } else {
                    BLUE_FLAG = 0;
                    RED_FLAG  = 1;
                }
                Task_Start(); // 开启巡线功能
                TFLAG = 1;
            } else if (result == 0x68) {
            }
        }
        /* ------------------- [红外启动] ------------------- */
        if (TFLAG == 0) {
            if (IR_CLEAR) IR_ARMED = 1; // 看到一次空闲，进入待命
            if (IR_ARMED && IR_DETECTED) {
                if (IR_COUNT < IR_CONFIRM_COUNT) IR_COUNT++;
                if (IR_COUNT >= IR_CONFIRM_COUNT) {
                    if (TEAM_CHOSE == 1) {
                        BLUE_FLAG = 1;
                        RED_FLAG  = 0;
                    } else {
                        BLUE_FLAG = 0;
                        RED_FLAG  = 1;
                    }
                    Task_Start(); // 启动巡线任务
                    TFLAG = 1;
                    TUBES_SetValue(TUBE_STARTED);
                }
            } else {
                IR_COUNT = 0;
            }
            if (TFLAG == 0) TUBES_SetValue(IR_DETECTED ? 0 : 1); // 未启动：显示引脚电平
            delay_ms(2); // 采样间隔，配合 IR_CONFIRM_COUNT 防抖
        }
        TUBES_Display();
        if (TFLAG == 0) {
            if (Direction == FRONT) {
                FRONT_BACK_TRACING((int8_t)(DRIVE_SIGN * SPEED), 1);
            }
            if (Direction == BACK) {
                FRONT_BACK_TRACING((int8_t)(-DRIVE_SIGN * SPEED), 0);
            }
            if (Direction == LEFT) {
                RIGHT_LEFT_TRACING((int8_t)(DRIVE_SIGN * SPEED), 1);
            }
            if (Direction == RIGHT) {
                RIGHT_LEFT_TRACING((int8_t)(-DRIVE_SIGN * SPEED), 0);
            }
            if (Direction == STOP) {
                FRONT_BACK_TRACING(0, 0);
            }
            if (Direction == REVOLVE) {
                Revolve(SPEED);
            }
        } else {
            if (Direction == FRONT) {
                FRONT_BACK_TRACING((int8_t)(DRIVE_SIGN * 15), 1);
            }
            if (Direction == BACK) {
                FRONT_BACK_TRACING((int8_t)(-DRIVE_SIGN * 15), 0);
            }
            if (Direction == LEFT) {
                RIGHT_LEFT_TRACING((int8_t)(DRIVE_SIGN * 15), 1);
            }
            if (Direction == RIGHT) {
                RIGHT_LEFT_TRACING((int8_t)(-DRIVE_SIGN * 15), 0);
            }
            if (Direction == STOP) {
                FRONT_BACK_TRACING(0, 0);
            }
            if (STEERING_FLAG) {
                Engine_Working();
            }
        }
    }
}
