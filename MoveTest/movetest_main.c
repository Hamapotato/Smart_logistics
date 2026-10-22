/* ============================================================================
 *  MoveTest  ——  底盘运动测试（修正轮速组合之后）
 *
 *  【已经测出来的结论】（2026-10-10，点名那一版）
 *      Motor_201 = 右前,  202 = 右后,  203 = 左后,  204 = 左前
 *      四个电机"正转"都是车轮前进方向 —— 四轮同向 = 车往前走
 *      驱动板认正负号（正转和反转是真的相反，不是只有大小）
 *      (+1,-1,-1,+1) 发下去轮子转、车不动  ==> 那一组是"空转"（对角轮对顶）
 *
 *  【由此算出来的组合】轮速空间是一组 Hadamard 基，四个组合两两正交：
 *
 *      前后平移 (+1,+1,+1,+1)     左右平移 (+1,-1,+1,-1)
 *      原地旋转 (+1,+1,-1,-1)     空转     (+1,-1,-1,+1)   <- 不产生任何位移
 *
 *      原地旋转为什么是 (+1,+1,-1,-1)：右前/右后两个轮一组、左后/左前两个轮一组，
 *      左右两组反向转。右侧轮推的力矩和左侧轮反向推的力矩同向叠加，合力为 0，
 *      所以就原地转。（正负号反过来就朝另一个方向转。）
 *
 *      **原来工程里把 (+1,-1,-1,+1) 当成"原地旋转"用了，那正好是空转组** ——
 *      这是这台车一直转不起来的直接原因。现在真旋转是 (+1,+1,-1,-1)。
 *
 *  【自动循环 4 步】每步节奏一样：
 *        绿灯闪 N 下      <- 报号（第几步，1..4）
 *        停    1 秒
 *        红灯常亮 2 秒    <- 这个组合的正方向
 *        停    1 秒
 *        红灯快闪 2 秒    <- 同一个组合的反方向
 *        停    2 秒
 *
 *      第 1 步 前进/后退   期望：往前开 / 往后开
 *      第 2 步 自转         期望：原地转（正反两个方向都该转，只是方向相反）
 *      第 3 步 平移         期望：横着平移（正反两个方向相反）
 *      第 4 步 空转         期望：轮子转，但**车不动**（对照组，验证上面的结论）
 *
 *      第 5..8 步是上次用过的"单个电机点名"，默认不跑；想单独看某个电机，
 *      把 MOVETEST_FROM / MOVETEST_TO 都设成它的编号（4..7）。
 *
 *  【重点看什么】
 *      第 2 步**有没有真的原地转起来** —— 这是这次改动的核心。
 *      转起来了，就把转的方向记下来（从上往下看是顺时针还是逆时针），
 *      正方向（红灯常亮）是哪个方向，好去定 Align 的 ALIGN_SIGN_YAW。
 *
 *  【故障提示】两个灯一起快闪 = I2C 往驱动板写失败了。写失败时程序会自动做一次
 *      总线恢复（SCL 补 9 个脉冲）再重试。这一步亮故障灯就说明板子在用旧指令。
 *
 *  【串口】19200 8N1（PA9 = TX，可不接）。每步开始打印第几步、期望什么。
 *
 *  【硬件】四轮驱动板 0x34 软 I2C  PF6 = SCL / PF5 = SDA
 *          绿灯 PA7 / 红灯 PA6（低电平点亮）。其它引脚一律不动。
 * ========================================================================== */

#include "main.h"

/* motor.c 里 MOTOR_Data[] 是全局的（motor.h 没声明它）。这里自己声明一份，
   就能绕过 MOTOR_CONTROL_DATA_Predit() 直接调 I2C_Write_Len() 并**拿到返回值**
   —— 那个函数把返回值丢掉了，所以写入失败在原来那条路上是看不见的。
   下标顺序必须和 MOTOR_CONTROL_DATA_Predit(&Motor_203,&Motor_202,&Motor_204,&Motor_201)
   内部一致： MOTOR_Data[0]=Motor_203, [1]=Motor_202, [2]=Motor_204, [3]=Motor_201。 */
extern int8_t MOTOR_Data[];

/* motor.c 里这两个也是全局的，但 motor.h 没声明。总线恢复要用它们切 SDA 方向。 */
extern void I2C_SDA_OUT(void);
extern void I2C_SDA_IN(void);

/* ---------------------------------------------------------------------------
 *  一、可调参数
 * ------------------------------------------------------------------------- */

/* 速度：写进 0x33 速度寄存器的值（闭环）。负号表示反向。 */
#define MOVETEST_SPEED      10

/* 每个方向持续多久 / 方向之间停多久 / 两步之间多停多久（毫秒） */
#define MOVETEST_RUN_MS     2000
#define MOVETEST_GAP_MS     1000
#define MOVETEST_EXTRA_MS   1000

/* 刷新周期：每 20ms 一个 tick。
   驱动板写完会一直保持这个转速，本来不用重发；但**万一某次写丢了，板子会一直
   保持上一次的转速**，所以周期性补发，顺便也能发现写失败。 */
#define MOVETEST_TICK_MS    20

/* 每多少个 tick 重发一次（5 * 20ms = 100ms）。想减少 I2C 流量就调大。 */
#define MOVETEST_SEND_TICKS 5

/* 一次下发最多试几次（失败会先做总线恢复再重试） */
#define MOVETEST_RETRY      3

/* 跑哪几步：0..7。默认跑 0..3（整车组合）。
   只想盯某一步：MOVETEST_FROM 和 MOVETEST_TO 都设成它的编号。 */
#define MOVETEST_FROM       0
#define MOVETEST_TO         3

/* 1 = 串口打印过程；0 = 串口完全不出声。 */
#define MOVETEST_DEBUG      1

/* 1 = 每个方向前后各读一次四路编码器，打印增量（能自动看出哪个电机在动）。
   默认关：这块板子对 0x3C 单通道读的支持没验证过，而一旦它把 I2C 总线搞死，
   正好会造成"一直在用旧指令"的现象 —— 那会污染测试结论。 */
#define MOVETEST_ENCODER    0

/* LED 低电平点亮（PA6/PA7 输出 0 是亮）。 */
#define LED_ON              0
#define LED_OFF             1

/* ---------------------------------------------------------------------------
 *  二、组合表（下标 0..7）
 * ------------------------------------------------------------------------- */

typedef struct {
    const char *name;       /* 串口里打印的名字 */
    const char *expect;     /* 应该看到什么 */
    int8_t      m[4];       /* 依次乘到 Motor_201 / 202 / 203 / 204 上 */
} MoveTest_Pattern_t;

static const MoveTest_Pattern_t MT_STEP[] = {
    /* ---- 0..3：整车组合，默认跑这四个 ---- */
    { "FWD_BACK", "car drives forward, then backward",
                    { +1, +1, +1, +1 } },
    { "SPIN",     "car spins in place (the two directions opposite)",
                    { +1, +1, -1, -1 } },
    { "STRAFE",   "car slides sideways (the two directions opposite)",
                    { +1, -1, +1, -1 } },
    { "NULL_OLD", "wheels turn but the CAR DOES NOT MOVE (old 'spin')",
                    { +1, -1, -1, +1 } },

    /* ---- 4..7：单个电机点名（已测过，默认不跑）---- */
    { "ONLY_M201", "only Motor_201 runs", { +1,  0,  0,  0 } },
    { "ONLY_M202", "only Motor_202 runs", {  0, +1,  0,  0 } },
    { "ONLY_M203", "only Motor_203 runs", {  0,  0, +1,  0 } },
    { "ONLY_M204", "only Motor_204 runs", {  0,  0,  0, +1 } },
};

#define MT_STEP_N   ((uint8_t)(sizeof(MT_STEP) / sizeof(MT_STEP[0])))

/* 指示灯用哪个 */
#define MT_LAMP_OFF   0   /* 两个都灭 */
#define MT_LAMP_POS   1   /* 红灯常亮：正方向 */
#define MT_LAMP_NEG   2   /* 红灯快闪：反方向 */
#define MT_LAMP_ERR   3   /* 双灯齐闪：I2C 写失败 */

/* ---------------------------------------------------------------------------
 *  三、代码
 * ------------------------------------------------------------------------- */

/* 本测试一行灰阶代码都没写，但工程的文件清单里带着四块灰阶的驱动
   (B/F/L/R_GREY_driver.c)，它们都调用 reverse()，而 reverse() 定义在
   APP/trace/trace.c —— 那个文件不在本工程里。所以这里必须自带一份，
   否则链接会报 L6218E: Undefined symbol reverse。
   （Test_Grey、Align 两个工程都是同样的绕法。）别当死代码删掉。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}

/* 总线恢复：从机把 SDA 拉死不放时，主机在 SCL 上补 9 个脉冲通常能让它把
   剩下的位吐完、自己松手；最后再补一个 STOP。不这样做的话，一旦总线卡住，
   后面每一次下发都是失败的，而板子会一直保持上一次的转速。 */
static void MT_I2C_Recover(void)
{
    uint8_t i;

    I2C_SDA_IN();               /* 先放开 SDA，别跟从机对着拉 */
    for (i = 0; i < 9; i++) {
        IIC_SCL = 0;
        delay_us(5);
        IIC_SCL = 1;
        delay_us(5);
    }
    I2C_SDA_OUT();              /* 补一个 STOP：SCL 高时 SDA 由低变高 */
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 1;
    delay_us(5);
    IIC_SDA = 1;
    delay_us(5);
}

/* 按第 idx 行组合、以 speed 驱动四个轮子。
   返回 0 = 写成功，1 = 几次重试都失败。 */
static uint8_t MT_Drive(uint8_t idx, int16_t speed)
{
    int16_t v[4];
    uint8_t i, r;

    for (i = 0; i < 4; i++) v[i] = (int16_t)(MT_STEP[idx].m[i] * speed);

    /* 顺序照抄 MOTOR_CONTROL_DATA_Predit()（错位的，故意的，别按直觉改） */
    MOTOR_Data[0] = (int8_t)v[2];   /* Motor_203 */
    MOTOR_Data[1] = (int8_t)v[1];   /* Motor_202 */
    MOTOR_Data[2] = (int8_t)v[3];   /* Motor_204 */
    MOTOR_Data[3] = (int8_t)v[0];   /* Motor_201 */

    for (r = 0; r < MOVETEST_RETRY; r++) {
        if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, MOTOR_COUNT) == 0)
            return 0;
        MT_I2C_Recover();
    }
    return 1;
}

/* 指示灯。tick 的单位是 MOVETEST_TICK_MS（20ms）。 */
static void MT_Lamp(uint8_t lamp, uint32_t tick)
{
    switch (lamp) {
        case MT_LAMP_POS:                       /* 常亮 */
            LED_RED   = LED_ON;
            LED_GREEN = LED_OFF;
            break;
        case MT_LAMP_NEG:                       /* 100ms 翻转一次 = 快闪 */
            LED_RED   = ((tick / 5) & 1) ? LED_ON : LED_OFF;
            LED_GREEN = LED_OFF;
            break;
        case MT_LAMP_ERR:                       /* 40ms 翻转一次 = 两个一起急闪 */
            LED_RED   = ((tick / 2) & 1) ? LED_ON : LED_OFF;
            LED_GREEN = ((tick / 2) & 1) ? LED_OFF : LED_ON;
            break;
        default:                                /* 全灭 */
            LED_RED   = LED_OFF;
            LED_GREEN = LED_OFF;
            break;
    }
}

/* 绿灯闪 n 下报号，最后多停一会儿，好让下一步的闪和这一步分得开。 */
static void MT_BlinkNum(uint8_t n)
{
    uint8_t i;

    for (i = 0; i < n; i++) {
        LED_GREEN = LED_ON;
        LED_RED   = LED_OFF;
        delay_ms(200);
        LED_GREEN = LED_OFF;
        delay_ms(250);
    }
    delay_ms(600);
}

/* 保持某个组合 ms 毫秒，按 MT_LAMP_* 亮灯，每 MOVETEST_SEND_TICKS 个 tick
   重发一次指令。写失败会亮故障灯并打印。返回失败的次数。 */
static uint16_t MT_Hold(uint8_t idx, int16_t speed, uint16_t ms, uint8_t lamp)
{
    uint32_t tick = 0;
    uint16_t t, bad = 0;
    uint8_t  send = 1;                  /* 进这一段必须先发一次 */

    for (t = 0; t < ms; t = (uint16_t)(t + MOVETEST_TICK_MS)) {
        if (send) {
            if (MT_Drive(idx, speed) != 0) bad++;
            send = 0;
        }
        MT_Lamp(lamp, tick);
        tick++;
        if ((tick % MOVETEST_SEND_TICKS) == 0) send = 1;
        delay_ms(MOVETEST_TICK_MS);
    }

    if (bad) {
        /* 写失败：亮 800ms 故障灯，别让人错过 */
        for (t = 0; t < 40; t++) {
            MT_Lamp(MT_LAMP_ERR, t);
            delay_ms(MOVETEST_TICK_MS);
        }
#if MOVETEST_DEBUG
        printf("MoveTest: *** I2C WRITE FAILED %d time(s) - board is running on a STALE command\r\n",
               (int)bad);
#endif
    }
    return bad;
}

/* 跑一整步：报号 -> 正 2s -> 停 -> 反 2s -> 停 */
static void MT_Step(uint8_t idx)
{
    uint16_t bad = 0;
#if MOVETEST_ENCODER
    int32_t e0[4], e1[4];
    uint8_t k;
#endif

#if MOVETEST_DEBUG
    printf("\r\nMoveTest: === step %d : %s -> %s ===\r\n",
           (int)(idx + 1), MT_STEP[idx].name, MT_STEP[idx].expect);
    printf("MoveTest: M201..M204 = %d %d %d %d.  green blinks %d x, then red SOLID = +%d, red BLINK = -%d\r\n",
           (int)MT_STEP[idx].m[0], (int)MT_STEP[idx].m[1],
           (int)MT_STEP[idx].m[2], (int)MT_STEP[idx].m[3],
           (int)(idx + 1), (int)MOVETEST_SPEED, (int)MOVETEST_SPEED);
#endif

    MT_BlinkNum((uint8_t)(idx + 1));            /* 报号 */
    MT_Hold(idx, 0, MOVETEST_GAP_MS, MT_LAMP_OFF);   /* 停稳再动 */

#if MOVETEST_ENCODER
    for (k = 0; k < 4; k++) e0[k] = MOTOR_Encoder_Read((uint8_t)(k + 1));
#endif

    bad += MT_Hold(idx, (int16_t)(+MOVETEST_SPEED), MOVETEST_RUN_MS, MT_LAMP_POS);

#if MOVETEST_ENCODER
    for (k = 0; k < 4; k++) e1[k] = MOTOR_Encoder_Read((uint8_t)(k + 1));
    printf("MoveTest: step %d  +dir enc M1..M4 delta = %d %d %d %d\r\n",
           (int)(idx + 1),
           (int)(e1[0] - e0[0]), (int)(e1[1] - e0[1]),
           (int)(e1[2] - e0[2]), (int)(e1[3] - e0[3]));
#endif

    MT_Hold(idx, 0, MOVETEST_GAP_MS, MT_LAMP_OFF);

#if MOVETEST_ENCODER
    for (k = 0; k < 4; k++) e0[k] = MOTOR_Encoder_Read((uint8_t)(k + 1));
#endif

    bad += MT_Hold(idx, (int16_t)(-MOVETEST_SPEED), MOVETEST_RUN_MS, MT_LAMP_NEG);

#if MOVETEST_ENCODER
    for (k = 0; k < 4; k++) e1[k] = MOTOR_Encoder_Read((uint8_t)(k + 1));
    printf("MoveTest: step %d  -dir enc M1..M4 delta = %d %d %d %d\r\n",
           (int)(idx + 1),
           (int)(e1[0] - e0[0]), (int)(e1[1] - e0[1]),
           (int)(e1[2] - e0[2]), (int)(e1[3] - e0[3]));
#endif

    MT_Hold(idx, 0, (uint16_t)(MOVETEST_GAP_MS + MOVETEST_EXTRA_MS), MT_LAMP_OFF);

#if MOVETEST_DEBUG
    if (bad == 0) printf("MoveTest: step %d done, all writes OK\r\n", (int)(idx + 1));
#endif
}

int main(void)
{
    uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);              /* delay_us / delay_ms 要用，必须最先做 */
    LED_Init();
    USART1_Init(19200);

    LED_RED   = LED_OFF;
    LED_GREEN = LED_OFF;

    MOTOR_Init();
    delay_ms(5);

    /* 上电第一件事：先让四个轮子停下，并且**检查这次写有没有成功**。
       不检查的话，一旦这次写丢了，板子会保持上次的转速 —— 上电就是"自己往前跑"。
       注意 delay_ms 的上限是 798ms（SysTick->LOAD 只有 24 位），别写大了。 */
    for (i = 0; i < 3; i++) {
        if (MT_Drive(0, 0) == 0) break;     /* 全 0 == 停车 */
        delay_ms(50);
    }

    delay_ms(500);

#if MOVETEST_DEBUG
    printf("\r\n== MoveTest (patterns) ==  speed=%d  each dir=%dms  steps=%d..%d\r\n",
           (int)MOVETEST_SPEED, (int)MOVETEST_RUN_MS,
           (int)MOVETEST_FROM, (int)MOVETEST_TO);
    printf("MoveTest: M201=FRONT-RIGHT M202=BACK-RIGHT M203=BACK-LEFT M204=FRONT-LEFT\r\n");
    printf("MoveTest: green blink count = step number (1-based).\r\n");
    printf("MoveTest: red SOLID = positive dir, red BLINK = negative dir.\r\n");
    printf("MoveTest: BOTH leds flashing = I2C write failed (stale command!).\r\n");
#endif

    for (;;) {
        for (i = MOVETEST_FROM; i <= MOVETEST_TO; i++) MT_Step((uint8_t)i);
    }
}
