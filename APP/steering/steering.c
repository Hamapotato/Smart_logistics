#include "steering.h"

/* ===========================================================================
 * 投递/抛出机构。
 *
 * ARM_USE_I2C_MOTOR = 1：JGB520 编码器减速电机，接在 0x34 驱动板的 ARM_CH
 *   路上（1~4 是四轮）。用编码器计数做位置闭环；行程终点靠堵转判定——驱动板
 *   内部是速度环，机械臂顶到限位后它拉不住转速，表现为「指令还在、计数不动」。
 *
 * ARM_USE_I2C_MOTOR = 0：原来的 MG996R 舵机（TIM9_CH1 / PE5，50Hz PWM）。
 *
 * 所有 I2C 访问都留在主循环（Steering_Task，由 main() 调用）。绝不能在中断里
 * 碰电机总线：四轮的指令也是在主循环里经同一条 PF5/PF6 软件 I2C 下发的，中断
 * 插进去做一次传输会把正在进行的 bit-bang 打乱。
 * =========================================================================== */

#if ARM_USE_I2C_MOTOR

/* 请求标志，由 Steering_RESET()/SET()/Home() 写入。注意 Steering_RESET() 会经
   Control_Data_Process() 从 TIM7 中断里被调到，所以这几个函数只能做普通变量
   赋值：不能碰 I2C，不能 printf。 */
#define ARM_REQ_NONE   0
#define ARM_REQ_HOME   1
#define ARM_REQ_THROW  2
#define ARM_REQ_RETURN 3

enum { ARM_IDLE = 0, ARM_HOMING, ARM_READY, ARM_THROW, ARM_RETURN, ARM_FAULT };

volatile u8 STEERING_FLAG = 0;

static volatile u8 arm_request;
static u8          arm_state;
static u8          arm_stall_cnt;
static u8          arm_zeroed;
static u8          arm_io_fail;
static u8          arm_fault_code;
static u32         arm_t_start;
static u32         arm_t_last;
static int32_t     arm_pos;
static int32_t     arm_vel; /* 每个标称 30ms 周期的计数变化量 */

/* 回零方向：复位位在计数 0，抛出让计数朝 ARM_THROW_COUNTS 增加，所以回零要往
   计数减小的方向爬。 */
#if (ARM_THROW_COUNTS >= 0)
#define ARM_HOME_SIGN  (-1)
#else
#define ARM_HOME_SIGN  (+1)
#endif

static int32_t i32abs(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* 把「计数单位」的指令（正 = 让计数变大）限幅后翻译成驱动板要的符号。 */
static int8_t arm_out(int32_t count_cmd)
{
    if (count_cmd > ARM_MAX_CMD) count_cmd = ARM_MAX_CMD;
    if (count_cmd < -ARM_MAX_CMD) count_cmd = -ARM_MAX_CMD;
    return (int8_t)(ARM_DIR_SIGN * count_cmd);
}

/* 计数环 PD。微分作用在「测得的」速度上而不是误差上，目标阶跃不会打出脉冲。 */
static int32_t arm_pd(int32_t err)
{
    float u = ARM_KP * (float)err - ARM_KD * (float)arm_vel;

    if (u > ARM_MAX_CMD) u = ARM_MAX_CMD;
    if (u < -ARM_MAX_CMD) u = -ARM_MAX_CMD;
    return (int32_t)u;
}

static void arm_fault(u8 code)
{
    Motor_205.PID_Output = 0;
    arm_fault_code       = code;
    arm_state            = ARM_FAULT;
    STEERING_FLAG        = 0; /* 放行规划器，别让比赛卡死 */
    printf("ARM_FAULT code=%d pos=%d\r\n", (int)arm_fault_code, (int)arm_pos);
}

void Steering_Init(void)
{
    /* 电机侧不需要 MCU 做什么：驱动板由 MOTOR_Init() 配置，也不再占用任何 PWM
       引脚（PE5 上那路 50Hz 舵机信号已经不用了）。 */
    arm_request   = ARM_REQ_NONE;
    arm_state     = ARM_IDLE;
    arm_stall_cnt = 0;
    arm_zeroed    = 0;
    arm_io_fail   = 0;
    arm_t_start   = 0;
    arm_t_last    = 0;
    arm_pos       = 0;
    arm_vel       = 0;
    Motor_205.PID_Output = 0;
}

void Steering_Home(void)
{
    arm_request   = ARM_REQ_HOME;
    STEERING_FLAG = 1;
}

void Steering_RESET(void)
{
    arm_request   = arm_zeroed ? ARM_REQ_RETURN : ARM_REQ_HOME;
    STEERING_FLAG = 1;
}

void Steering_SET(void)
{
    arm_request   = arm_zeroed ? ARM_REQ_THROW : ARM_REQ_HOME;
    STEERING_FLAG = 1;
}

/* 保留这个名字给 main.c 里 `if (STEERING_FLAG) Engine_Working();` 那个调用点。
   电机方案下所有活都在 Steering_Task() 里干。 */
void Engine_Working(void)
{
}

void Steering_Task(void)
{
    u8      req;
    u32     dt;
    int32_t pos;
    int32_t err;
    int32_t cmd;

    if (!TIM7_TICK_FLAG) return; /* 全部动作对齐到 TIM7 的 30ms 节拍 */
    TIM7_TICK_FLAG = 0;

    /* ---- 取一个待处理请求 ---- */
    req = arm_request;
    if (req != ARM_REQ_NONE) {
        arm_request = ARM_REQ_NONE;
        if ((arm_state == ARM_IDLE) || (arm_state == ARM_READY) || (arm_state == ARM_FAULT)) {
            arm_stall_cnt = 0;
            arm_t_start   = TIM7_Tick_MS;
            arm_t_last    = TIM7_Tick_MS; /* 别用上一个动作的旧时戳算 dt */
            if (req == ARM_REQ_HOME) {
                arm_state = ARM_HOMING;
            } else if (!arm_zeroed) {
                arm_state = ARM_HOMING; /* 还没找到复位位，先回零 */
            } else if (req == ARM_REQ_THROW) {
                arm_state = ARM_THROW;
            } else {
                arm_state = ARM_RETURN;
            }
            STEERING_FLAG = 1;
        }
        /* 机构正忙时不打断当前动作，请求丢弃；忙完自然降旗。 */
    }

    if ((arm_state == ARM_IDLE) || (arm_state == ARM_READY)) {
        Motor_205.PID_Output = 0;
        /* 空闲时不碰 STEERING_FLAG：降旗由本模块的动作完成路径和 arm_fault()
           负责。在这里清会把 move.c 刚到点置起的抛投请求提前放行一个周期。 */
        return;
    }
    if (arm_state == ARM_FAULT) {
        Motor_205.PID_Output = 0;
        STEERING_FLAG        = 0;
        return;
    }

    /* ---- 读编码器（只在主循环里做） ---- */
    pos = MOTOR_Encoder_Read(ARM_CH);
    if (pos == MOTOR_ENCODER_ERR) {
        if (++arm_io_fail >= ARM_IO_FAIL_MAX) arm_fault(1);
        return; /* 保留上一次位置继续驱动，不因为一次读失败就停机 */
    }
    arm_io_fail = 0;

    dt = TIM7_Tick_MS - arm_t_last;
    if (dt == 0) dt = 1;
    arm_t_last = TIM7_Tick_MS;
    /* 归一化成「每 30ms 的计数」，这样主循环偶尔被拖慢也不会改变 KD 项的
       实际强度。 */
    arm_vel = (int32_t)((pos - arm_pos) * 30 / (int32_t)dt);
    arm_pos = pos;

    /* 回零要从抛出位一路爬回复位位，距离可能很长，给它单独的超时。 */
    if ((TIM7_Tick_MS - arm_t_start) >
        ((arm_state == ARM_HOMING) ? ARM_HOME_TIMEOUT_MS : ARM_TIMEOUT_MS)) {
        arm_fault(2);
        return;
    }

    switch (arm_state) {

    case ARM_HOMING:
        /* 低速往复位位爬；指令还在、计数却不动 = 顶到限位了，把这位置作 0。 */
        if (i32abs(arm_vel) <= ARM_STALL_EPS) {
            if (++arm_stall_cnt >= ARM_STALL_N) {
                Motor_205.PID_Output = 0;
                arm_pos              = 0;
                arm_vel              = 0;
                arm_zeroed           = 1;
                arm_state            = ARM_READY;
                STEERING_FLAG        = 0;
                break;
            }
        } else {
            arm_stall_cnt = 0;
        }
        Motor_205.PID_Output = arm_out(ARM_HOME_SIGN * ARM_HOME_CMD);
        break;

    case ARM_THROW:
        /* 到抛出位、或顶到抛出方向的硬限位，谁先到谁算完事。 */
        err = ARM_THROW_COUNTS - arm_pos;
        if (i32abs(err) <= ARM_ARRIVE_TOL) {
            Motor_205.PID_Output = 0;
            arm_stall_cnt        = 0;
            arm_t_start          = TIM7_Tick_MS;
            arm_state            = ARM_RETURN;
            break;
        }
        cmd = arm_pd(err);
        if ((i32abs(arm_vel) <= ARM_STALL_EPS) && (i32abs(cmd) >= ARM_STALL_CMD)) {
            if (++arm_stall_cnt >= ARM_STALL_N) {
                Motor_205.PID_Output = 0;
                arm_stall_cnt        = 0;
                arm_t_start          = TIM7_Tick_MS;
                arm_state            = ARM_RETURN;
                break;
            }
        } else {
            arm_stall_cnt = 0;
        }
        Motor_205.PID_Output = arm_out(cmd);
        break;

    case ARM_RETURN:
        err = -arm_pos;
        if (i32abs(err) <= ARM_ARRIVE_TOL) {
            Motor_205.PID_Output = 0;
            arm_state            = ARM_READY;
            STEERING_FLAG        = 0;
            break;
        }
        cmd = arm_pd(err);
        if ((i32abs(arm_vel) <= ARM_STALL_EPS) && (i32abs(cmd) >= ARM_STALL_CMD)) {
            if (++arm_stall_cnt >= ARM_STALL_N) {
                /* 压在复位挡块上，那这里就是 0：顺手重新标零，把每圈的累积
                   误差吃掉。 */
                Motor_205.PID_Output = 0;
                arm_pos              = 0;
                arm_vel              = 0;
                arm_state            = ARM_READY;
                STEERING_FLAG        = 0;
                break;
            }
        } else {
            arm_stall_cnt = 0;
        }
        Motor_205.PID_Output = arm_out(cmd);
        break;

    default:
        break;
    }
}

#else /* ------------- 旧方案：MG996R 舵机（TIM9_CH1 / PE5） ---------------- */

volatile u8 STEERING_FLAG = 0; //0:RESET 1:SET
u16 Steering_Delay = 0;

void Steering_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 168-1;
    TIM_TimeBaseStructure.TIM_Period = 20000-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);

    TIM_Cmd(TIM9, ENABLE);
}

void Steering_RESET(void)
{
    TIM_SetCompare1(TIM9, 800);
}

void Steering_SET(void)
{
    TIM_SetCompare1(TIM9, 1700);
}

void Engine_Working(void)
{
    if (Steering_Delay == 0) {
        Steering_SET();
        Steering_Delay++;
    } else if (Steering_Delay == 1000) {
        STEERING_FLAG = 0;
        Steering_Delay++;
    } else if (Steering_Delay == 1001) {
        Steering_RESET();
        Steering_Delay++;
    } else if (Steering_Delay == 1500) {
        STEERING_FLAG = 0;
        Steering_Delay = 0;
    } else {
        Steering_Delay++;
    }
}

void Steering_Home(void)
{
    Steering_RESET();
}

void Steering_Task(void)
{
}

#endif
