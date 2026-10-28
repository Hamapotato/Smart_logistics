/* ============================================================================
 * SpeedTest v2 —— JGB37-520 四轮编码器速度闭环测试
 *
 * 【与 MoveTest 相同的下发方式】
 *   使用驱动板 0x33 闭环速度寄存器，直接向四路写有符号目标速度，
 *   不再使用 0x1F 固定 PWM。MoveTest 已证明 0x33 在当前板上工作正常。
 *
 * 【MCU 外环】
 *   读取 0x3C 四路编码器 -> 计算实际编码器速度
 *   -> 与底盘目标速度对应的编码器速度比较
 *   -> PI 修正 0x33 的四路速度指令。
 *
 *   因此控制结构是：
 *       底盘目标速度 -> MCU 外环 -> 0x33 目标脉冲/10ms
 *                              -> 驱动板内部速度环 -> 电机
 *       编码器 0x3C -------------------------------------^
 *
 * 【电机参数】
 *   JGB37-520，330 r/min，减速比 1:30，默认 11 线 Hall、四倍频：
 *       CPR = 11 × 4 × 30 = 1320 counts/输出轴圈
 *
 *   如果不是 11 线编码器，用 SPEED_OUTPUT_ENABLE=0 手动转一圈，
 *   根据串口 pos201..pos204 的前后差值重新填写 SPEED_ENCODER_CPR。
 *
 * 【Keil 里主要修改】
 *       SPEED_MODE
 *       SPEED_TARGET_MM_S
 *       SPEED_ENCODER_CPR
 *       SPEED_KP / SPEED_KI
 * ========================================================================== */

#include "main.h"

/* ---------------------------------------------------------------------------
 *  一、可调参数
 * ------------------------------------------------------------------------- */

#define SPEED_MODE_FRONT    0
#define SPEED_MODE_BACK     1
#define SPEED_MODE_LEFT     2
#define SPEED_MODE_RIGHT    3
#define SPEED_MODE_SPIN     4

/* 只改这一行选择测试方向。 */
#define SPEED_MODE          SPEED_MODE_FRONT

/* 底盘平移目标速度，单位 mm/s。
   原地旋转模式中，这个值表示每个轮子的切向线速度。 */
#define SPEED_TARGET_MM_S   200

#define SPEED_WHEEL_DIAMETER_MM   58.0f

/* 45° 对角全向轮：底盘平移速度到单轮线速度约为 1/sqrt(2)。 */
#define SPEED_OMNI_TO_WHEEL_FACTOR   0.70710678f

/* JGB37-520 11 线 Hall、四倍频、减速比 1:30。 */
#define SPEED_ENCODER_CPR         1320.0f

/* 标称控制周期。实际时间用 TIM2 测量。 */
#define SPEED_TICK_MS             50

/* 上电等待时间。 */
#define SPEED_START_DELAY_MS      800

/* 1 = 编码器 PI 外环；0 = 仅使用目标速度直接写 0x33，
   用于确认 MoveTest 式下发是否正常。 */
#define SPEED_USE_OUTER_PI        1

/* PI 参数，输出单位与 0x33 速度指令相同。 */
#define SPEED_KP                  1.00f
#define SPEED_KI                  0.05f

/* 0x33 单路指令限幅。有效范围按有符号 int8，保守限制到 ±80。 */
#define SPEED_CMD_LIMIT           80

/* PI 积分限幅。 */
#define SPEED_INTEGRAL_LIMIT      300.0f

/* 实测速度误差小于这个值就当 0，单位 counts/10ms。 */
#define SPEED_DEADBAND            0.5f

/* 编码器无变化保护：
   有速度指令，但连续 N 个周期几乎测不到编码器变化，就认为该轮编码器
   没有反馈，停止外环积分并退回“目标速度前馈”，避免该轮被外环越推越快。 */
#define SPEED_ENC_STALL_EPS       0.8f
#define SPEED_ENC_STALL_N         10

/* 1 = 电机闭环；0 = 电机停，只读编码器用于 CPR 标定。 */
#define SPEED_OUTPUT_ENABLE       1

/* I2C 下发最多重试次数。 */
#define SPEED_RETRY               3

/* 每多少个周期打印一次。 */
#define SPEED_PRINT_EVERY         10

#define SPEED_DEBUG               1

#define SPEED_LED_ON              0
#define SPEED_LED_OFF             1

/* 单轮编码器方向。若目标速度为正、轮子也为正转，
   但该轮编码器实测速度长期为负，就把它改成 -1。 */
#define SPEED_ENC_SIGN_201        (+1)
#define SPEED_ENC_SIGN_202        (+1)
#define SPEED_ENC_SIGN_203        (+1)
#define SPEED_ENC_SIGN_204        (+1)

#if (SPEED_TARGET_MM_S <= 0)
#error "SPEED_TARGET_MM_S must be positive"
#endif

#if (SPEED_MODE < SPEED_MODE_FRONT) || (SPEED_MODE > SPEED_MODE_SPIN)
#error "SPEED_MODE must be one of FRONT/BACK/LEFT/RIGHT/SPIN"
#endif

#if (SPEED_TICK_MS < 10)
#error "SPEED_TICK_MS is too short for software I2C"
#endif

#if (SPEED_CMD_LIMIT <= 0) || (SPEED_CMD_LIMIT > 100)
#error "SPEED_CMD_LIMIT must be in 1..100"
#endif

/* ---------------------------------------------------------------------------
 *  二、通道映射
 *
 *  车身顺序：201=右前, 202=右后, 203=左后, 204=左前
 *  驱动板顺序：CH1=203, CH2=202, CH3=204, CH4=201
 * ------------------------------------------------------------------------- */

extern int8_t MOTOR_Data[];
extern void I2C_SDA_OUT(void);
extern void I2C_SDA_IN(void);

static const uint8_t SPEED_BODY_TO_BOARD[4] = {
    3,  /* 201 -> CH4 */
    1,  /* 202 -> CH2 */
    0,  /* 203 -> CH1 */
    2   /* 204 -> CH3 */
};

static const int8_t SPEED_ENC_SIGN[4] = {
    SPEED_ENC_SIGN_201,
    SPEED_ENC_SIGN_202,
    SPEED_ENC_SIGN_203,
    SPEED_ENC_SIGN_204
};

static const int8_t SPEED_MODE_SIGN[5][4] = {
    { +1, +1, +1, +1 },  /* 前进 */
    { -1, -1, -1, -1 },  /* 后退 */
    { +1, -1, +1, -1 },  /* 左移 */
    { -1, +1, -1, +1 },  /* 右移 */
    { +1, +1, -1, -1 }   /* 原地旋转 */
};

static const char *const SPEED_MODE_NAME[5] = {
    "FRONT", "BACK", "LEFT", "RIGHT", "SPIN"
};

/* 通过 SWD 实时观察的调试镜像。它们不影响控制逻辑。 */
volatile uint32_t SpeedDbgLoopCount;
volatile uint32_t SpeedDbgReadOk;
volatile uint32_t SpeedDbgWriteOk;
volatile uint32_t SpeedDbgFlags;
volatile int32_t  SpeedDbgEnc[4];
volatile int32_t  SpeedDbgDelta[4];
volatile int16_t  SpeedDbgCmd[4];
volatile int32_t  SpeedDbgTarget10x100;
volatile uint32_t SpeedDbgI2cError;

typedef struct
{
    int32_t prev_raw;
    int32_t delta;
    float integral;
    int16_t command;
    uint8_t stall_count;
    uint8_t sign_error;
    uint8_t encoder_stall;
} SpeedWheelTypeDef;

/* ---------------------------------------------------------------------------
 *  三、基础工具
 * ------------------------------------------------------------------------- */

static float Speed_AbsF(float v)
{
    return (v < 0.0f) ? -v : v;
}

static int16_t Speed_ClampCommand(float v)
{
    if (v > (float)SPEED_CMD_LIMIT) v = (float)SPEED_CMD_LIMIT;
    if (v < -(float)SPEED_CMD_LIMIT) v = -(float)SPEED_CMD_LIMIT;
    return (int16_t)v;
}

/* TIM2 以 10 kHz 自由运行。SysTick 会被 delay_us() 反复改写，不能用于计时。 */
static void Speed_InitSampleTimer(void)
{
    TIM_TimeBaseInitTypeDef timer;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    timer.TIM_Prescaler     = 8400 - 1;
    timer.TIM_Period        = 0xFFFFFFFFU;
    timer.TIM_CounterMode   = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

static void Speed_I2C_Recover(void)
{
    uint8_t i;

    I2C_SDA_IN();
    for (i = 0; i < 9; i++)
    {
        IIC_SCL = 0;
        delay_us(5);
        IIC_SCL = 1;
        delay_us(5);
    }
    I2C_SDA_OUT();
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 1;
    delay_us(5);
    IIC_SDA = 1;
    delay_us(5);
}

/* board[0..3] = 驱动板 CH1..CH4。
 *
 * 这块板的 0x3C 整块 16 字节读取在实车上没有稳定成功，但按通道
 * 读 4 字节工作正常。分别读取：
 *     CH1 0x3C, CH2 0x40, CH3 0x44, CH4 0x48
 */
static uint8_t Speed_ReadEncoders(int32_t board[4])
{
    uint8_t buf[4];
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        uint32_t v;
        if (I2C_Read_Len(MOTOR_ENCODER_CH_ADDR(i + 1), buf, 4) != 0)
            return 0;

        v = (uint32_t)buf[0]
          | ((uint32_t)buf[1] << 8)
          | ((uint32_t)buf[2] << 16)
          | ((uint32_t)buf[3] << 24);
        board[i] = (int32_t)v;
    }
    return 1;
}

/* command[] 使用车身顺序 201,202,203,204；写入方式与 MoveTest 一致：0x33。 */
static uint8_t Speed_WriteCommand(const int16_t command[4])
{
    uint8_t r;

    MOTOR_Data[0] = (int8_t)command[2]; /* CH1 = 203 */
    MOTOR_Data[1] = (int8_t)command[1]; /* CH2 = 202 */
    MOTOR_Data[2] = (int8_t)command[3]; /* CH3 = 204 */
    MOTOR_Data[3] = (int8_t)command[0]; /* CH4 = 201 */

    for (r = 0; r < SPEED_RETRY; r++)
    {
        if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, 4) == 0)
            return 0;
        Speed_I2C_Recover();
    }
    return 1;
}

static void Speed_Stop(void)
{
    int16_t zero[4] = { 0, 0, 0, 0 };
    Speed_WriteCommand(zero);
}

static void Speed_ResetState(const int32_t board[4], SpeedWheelTypeDef wheel[4])
{
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        wheel[i].prev_raw  = board[SPEED_BODY_TO_BOARD[i]];
        wheel[i].delta     = 0;
        wheel[i].integral      = 0.0f;
        wheel[i].command       = 0;
        wheel[i].stall_count   = 0;
        wheel[i].sign_error    = 0;
        wheel[i].encoder_stall = 0;
    }
}

static float Speed_TargetScale(void)
{
    return (SPEED_MODE == SPEED_MODE_SPIN) ? 1.0f : SPEED_OMNI_TO_WHEEL_FACTOR;
}

/* 底盘目标速度 -> 每个轮子的目标编码器速度，单位 counts/10ms。 */
static float Speed_TargetRate10(void)
{
    return (float)SPEED_TARGET_MM_S
         * Speed_TargetScale()
         * SPEED_ENCODER_CPR
         / (3.14159265f * SPEED_WHEEL_DIAMETER_MM * 100.0f);
}

/* 外环 PI：目标值和实测值都是带方向的 counts/10ms。 */
static int16_t Speed_OuterPi(float target_rate10, float actual_rate10,
                             SpeedWheelTypeDef *wheel)
{
    float err = target_rate10 - actual_rate10;
    float u;

    if (Speed_AbsF(err) <= SPEED_DEADBAND) err = 0.0f;

    wheel->integral += err;
    if (wheel->integral > SPEED_INTEGRAL_LIMIT)
        wheel->integral = SPEED_INTEGRAL_LIMIT;
    if (wheel->integral < -SPEED_INTEGRAL_LIMIT)
        wheel->integral = -SPEED_INTEGRAL_LIMIT;

    /* 前馈就是目标 0x33 速度；PI 只修正编码器测到的误差。 */
    u = target_rate10 + SPEED_KP * err + SPEED_KI * wheel->integral;

    if ((u > (float)SPEED_CMD_LIMIT) && (err > 0.0f))
        wheel->integral -= err;
    if ((u < -(float)SPEED_CMD_LIMIT) && (err < 0.0f))
        wheel->integral -= err;

    return Speed_ClampCommand(u);
}

/* ---------------------------------------------------------------------------
 *  四、主程序
 * ------------------------------------------------------------------------- */

int main(void)
{
    int32_t board_enc[4];
    SpeedWheelTypeDef wheel[4];
    int16_t command[4] = { 0, 0, 0, 0 };
    float target_rate10;
    uint16_t tick = 0;
    uint32_t now_tick;
    uint32_t prev_tick;
    uint32_t dt_ticks;
    uint8_t read_fail = 0;
    uint8_t output_fail = 0;
    uint8_t sign_error = 0;
    uint8_t encoder_error = 0;
    uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    Speed_InitSampleTimer();
    LED_Init();
    USART1_Init(19200);

    LED_RED   = SPEED_LED_OFF;
    LED_GREEN = SPEED_LED_OFF;

    MOTOR_Init();
    Speed_Stop();
    delay_ms(100);

    SpeedDbgReadOk = 0;
    while (Speed_ReadEncoders(board_enc) == 0)
    {
        SpeedDbgLoopCount++;
        LED_RED = SPEED_LED_ON;
        delay_ms(100);
        LED_RED = SPEED_LED_OFF;
        delay_ms(100);
    }
    Speed_ResetState(board_enc, wheel);

    target_rate10 = Speed_TargetRate10();

#if SPEED_DEBUG
    printf("\r\n== SpeedTest v2: JGB37-520 encoder outer loop ==\r\n");
    printf("SpeedTest: mode=%s target=%dmm/s target_rate=%d counts/10ms\r\n",
           SPEED_MODE_NAME[SPEED_MODE], (int)SPEED_TARGET_MM_S,
           (int)(target_rate10 + 0.5f));
    printf("SpeedTest: CPR=%d KP=%.2f KI=%.2f outer_pi=%d output=%d\r\n",
           (int)SPEED_ENCODER_CPR, (double)SPEED_KP, (double)SPEED_KI,
           (int)SPEED_USE_OUTER_PI, (int)SPEED_OUTPUT_ENABLE);
    printf("SpeedTest: motor command register=0x33, encoder register=0x3C\r\n");
#endif

#if SPEED_OUTPUT_ENABLE
    delay_ms(SPEED_START_DELAY_MS);
#endif

    prev_tick = TIM2->CNT;

    for (;;)
    {
        if (Speed_ReadEncoders(board_enc) == 0)
        {
            SpeedDbgReadOk = 0;
            SpeedDbgLoopCount++;
            SpeedDbgI2cError = I2C_Read_LastError;
            SpeedDbgFlags = 0x00000004U;
            read_fail = 1;
            prev_tick = TIM2->CNT;
            Speed_Stop();
            LED_RED   = ((tick / 2) & 1) ? SPEED_LED_OFF : SPEED_LED_ON;
            LED_GREEN = SPEED_LED_OFF;
            delay_ms(SPEED_TICK_MS);
            tick++;
            continue;
        }

        SpeedDbgReadOk = 1;
        if (read_fail)
        {
            Speed_ResetState(board_enc, wheel);
            read_fail = 0;
        }
        for (i = 0; i < 4; i++) SpeedDbgEnc[i] = board_enc[i];

        now_tick = TIM2->CNT;
        dt_ticks = now_tick - prev_tick;
        prev_tick = now_tick;
        if (dt_ticks < 100U) dt_ticks = 100U;

        sign_error = 0;
        encoder_error = 0;

        for (i = 0; i < 4; i++)
        {
            uint8_t bi = SPEED_BODY_TO_BOARD[i];
            int32_t raw = board_enc[bi];
            float signed_target;
            float actual_rate10;

            wheel[i].delta = (raw - wheel[i].prev_raw) * SPEED_ENC_SIGN[i];
            wheel[i].prev_raw = raw;
            SpeedDbgDelta[i] = wheel[i].delta;

            actual_rate10 = (float)wheel[i].delta * 10000.0f / (float)dt_ticks;
            signed_target = target_rate10 * (float)SPEED_MODE_SIGN[SPEED_MODE][i];

#if SPEED_OUTPUT_ENABLE
            if ((Speed_AbsF(wheel[i].command) > 2.0f)
                && (Speed_AbsF(actual_rate10) < SPEED_ENC_STALL_EPS))
            {
                if (wheel[i].stall_count < 255U) wheel[i].stall_count++;
            }
            else if (Speed_AbsF(actual_rate10) > 1.0f)
            {
                wheel[i].stall_count = 0;
                wheel[i].encoder_stall = 0;
            }

            if (wheel[i].stall_count >= SPEED_ENC_STALL_N)
            {
                wheel[i].encoder_stall = 1;
                wheel[i].integral = 0.0f;
            }
            if (wheel[i].encoder_stall) encoder_error = 1;

            wheel[i].sign_error = 0;
            if ((Speed_AbsF(signed_target) > 1.0f)
                && (Speed_AbsF(actual_rate10) > 1.0f)
                && ((signed_target * actual_rate10) < 0.0f))
            {
                wheel[i].sign_error = 1;
                sign_error = 1;
                wheel[i].integral = 0.0f;
            }

#if SPEED_USE_OUTER_PI
            if (wheel[i].sign_error || wheel[i].encoder_stall)
            {
                /* 编码器方向不对或没有反馈时先保持 MoveTest 式前馈，
                   不再让外环把该轮指令越推越大。 */
                wheel[i].command = Speed_ClampCommand(signed_target);
            }
            else
            {
                wheel[i].command = Speed_OuterPi(signed_target, actual_rate10, &wheel[i]);
            }
#else
            wheel[i].command = Speed_ClampCommand(signed_target);
#endif
            command[i] = wheel[i].command;
#else
            wheel[i].integral = 0.0f;
            wheel[i].command = 0;
            command[i] = 0;
#endif
        }

#if SPEED_OUTPUT_ENABLE
        for (i = 0; i < 4; i++) SpeedDbgCmd[i] = command[i];
        SpeedDbgTarget10x100 = (int32_t)(target_rate10 * 100.0f + 0.5f);
        SpeedDbgFlags = (uint32_t)(sign_error ? 1U : 0U)
                      | (uint32_t)(encoder_error ? 2U : 0U);

        if (Speed_WriteCommand(command) != 0)
        {
            SpeedDbgWriteOk = 0;
            SpeedDbgFlags |= 0x00000004U;
            output_fail = 1;
            LED_RED   = ((tick / 2) & 1) ? SPEED_LED_OFF : SPEED_LED_ON;
            LED_GREEN = SPEED_LED_OFF;
        }
        else
        {
            SpeedDbgWriteOk = 1;
            output_fail = 0;
            LED_RED   = (sign_error || encoder_error)
                        ? (((tick / 2) & 1) ? SPEED_LED_OFF : SPEED_LED_ON)
                        : SPEED_LED_OFF;
            LED_GREEN = (sign_error || encoder_error) ? SPEED_LED_OFF : SPEED_LED_ON;
        }
#else
        Speed_Stop();
        LED_RED   = SPEED_LED_OFF;
        LED_GREEN = ((tick / 5) & 1) ? SPEED_LED_OFF : SPEED_LED_ON;
#endif

#if SPEED_DEBUG
        if ((tick % SPEED_PRINT_EVERY) == 0)
        {
            printf("Speed[%s] target=%dmm/s rate10=%d dt=%.1fms | d201=%ld d202=%ld d203=%ld d204=%ld | pos=%ld,%ld,%ld,%ld | cmd=%d,%d,%d,%d%s%s%s\r\n",
                   SPEED_MODE_NAME[SPEED_MODE], (int)SPEED_TARGET_MM_S,
                   (int)(target_rate10 + 0.5f), (double)((float)dt_ticks / 10.0f),
                   (long)wheel[0].delta, (long)wheel[1].delta,
                   (long)wheel[2].delta, (long)wheel[3].delta,
                   (long)board_enc[SPEED_BODY_TO_BOARD[0]],
                   (long)board_enc[SPEED_BODY_TO_BOARD[1]],
                   (long)board_enc[SPEED_BODY_TO_BOARD[2]],
                   (long)board_enc[SPEED_BODY_TO_BOARD[3]],
                   (int)command[0], (int)command[1],
                   (int)command[2], (int)command[3],
                   sign_error ? " | SIGN_ERR" : "",
                   encoder_error ? " | ENC_ERR" : "",
                   output_fail ? " | I2C_ERR" : "");
        }
#endif

        SpeedDbgLoopCount++;
        tick++;
        delay_ms(SPEED_TICK_MS);
    }
}

/* 工程清单包含灰阶驱动，它们引用 reverse()。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}
