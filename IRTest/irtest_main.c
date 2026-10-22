/* ============================================================================
 *  IRTest —— 四方向红外避障测试
 *
 *  【测试逻辑】
 *      哪一侧的红外检测到东西，车就朝相反方向平移：
 *
 *          前面检测到 -> 后退        后面检测到 -> 前进
 *          左侧检测到 -> 右移        右侧检测到 -> 左移
 *
 *      每一侧有两只探头，任意一只触发就算这一侧有障碍。
 *      同时有多侧触发时按矢量合成：
 *          前 + 左 -> 后退 + 右移（斜着退，轮速减半）
 *          前 + 后 -> 前后抵消，这一轴停
 *          左 + 右 -> 左右抵消，这一轴停
 *      四侧都没有遮挡 -> 四轮停。
 *
 *  【硬件】八路红外（2026-10-20 实测接线）：
 *      前左 PD13      前右 PE12
 *      后左 PB7       后右 PE4
 *      左前 PD14      左后 PB8
 *      右前 PE11      右后 PE3
 *
 *      八路都是 GPIO 输入 + 上拉，默认低电平 = 检测到障碍。
 *      如果实测极性相反（没有东西也一直触发），把 IR_ACTIVE_LOW 改成 0。
 *
 *      四轮板  软 I2C  PF6 = SCL, PF5 = SDA, 地址 0x34
 *      串口    USART1  PA9 = TX, 19200 8N1
 *      绿灯 PA7 / 红灯 PA6（低电平点亮）
 *
 *  【轮速组合】（MoveTest 已实测确认）
 *      201 = 右前, 202 = 右后, 203 = 左后, 204 = 左前
 *      前进 (+1,+1,+1,+1)   后退 (-1,-1,-1,-1)
 *      左移 (+1,-1,+1,-1)   右移 (-1,+1,-1,+1)
 *
 *      本程序只做平移，不转向。把前进量和左移量分开算：
 *          m201 = vx + vy    m202 = vx - vy
 *          m203 = vx + vy    m204 = vx - vy
 *      其中 vx > 0 前进、vx < 0 后退；vy > 0 左移、vy < 0 右移。
 * ========================================================================== */


#include "main.h"

/* ---------------------------------------------------------------------------
 *  一、可调参数
 * ------------------------------------------------------------------------- */

/* 平移速度。太小克服不了静摩擦，车会不动；第一次上车先用 15。 */
#define IR_SPEED            15

/* 控制周期，ms。 */
#define IR_TICK_MS          20

/* 一侧连续多少个周期都检测到才认（2 x 20ms = 40ms 去抖）。 */
#define IR_DEBOUNCE         2

/* 上电后先停多久再响应，ms。给人留一点时间把手和障碍物拿开。 */
#define IR_START_DELAY_MS   1000

/* 每多少个周期打一行串口（25 x 20ms = 500ms）。 */
#define IR_PRINT_EVERY      25

/* 1 = 串口输出过程；0 = 串口完全不出声（LED 照常）。 */
#define IR_DEBUG            1

/* 一次下发失败后的重试次数（失败先做总线恢复再重试）。 */
#define IR_RETRY            3

/* 红外模块输出极性：1 = 低电平表示检测到障碍（这类模块常见）。 */
#define IR_ACTIVE_LOW       1

/* 平移方向符号。若整组前后反了（前面有障碍反而往前顶），把 VX 改成 -1；
   左右反了就把 VY 改成 -1。正常情况下两行都不用动。 */
#define IR_SIGN_VX          (+1)
#define IR_SIGN_VY          (+1)

/* LED 低电平点亮。 */
#define IR_LED_ON           0
#define IR_LED_OFF          1

#if (IR_SPEED <= 0)
#error "IR_SPEED must be > 0"
#endif

#if (IR_DEBOUNCE < 1)
#error "IR_DEBOUNCE must be >= 1"
#endif

/* ---------------------------------------------------------------------------
 *  二、八路红外
 * ------------------------------------------------------------------------- */

#if IR_ACTIVE_LOW
  #define IR_READ(port, pin)    (!GPIO_ReadInputDataBit((port), (pin)))
#else
  #define IR_READ(port, pin)    (GPIO_ReadInputDataBit((port), (pin)) != 0)
#endif

/* 实测接线（2026-10-20）：
 *   前左 PD13   前右 PE12
 *   后左 PB7    后右 PE4
 *   左前 PD14   左后 PB8
 *   右前 PE11   右后 PE3
 * 某一根接反了只改对应的 IR_READ 行，逻辑不用动。 */
#define IR_FRONT_L      IR_READ(GPIOD, GPIO_Pin_13)
#define IR_FRONT_R      IR_READ(GPIOE, GPIO_Pin_12)
#define IR_BACK_L       IR_READ(GPIOB, GPIO_Pin_7)
#define IR_BACK_R       IR_READ(GPIOE, GPIO_Pin_4)
#define IR_LEFT_U       IR_READ(GPIOD, GPIO_Pin_14)
#define IR_LEFT_D       IR_READ(GPIOB, GPIO_Pin_8)
#define IR_RIGHT_U      IR_READ(GPIOE, GPIO_Pin_11)
#define IR_RIGHT_D      IR_READ(GPIOE, GPIO_Pin_3)

/* 每一侧的去抖状态。cnt 是连续"看到东西"的周期数。 */
typedef struct
{
    uint8_t cnt;
    uint8_t on;
} IR_SideTypeDef;

/* 八路红外：GPIO 输入 + 上拉（PB7/PB8、PD13/PD14、PE2/PE3/PE11/PE12）。 */
static void IR_SensorInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;

    gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOE, &gpio);
}

/* 一侧的两只探头任意一只触发就传给这里。带一点迟滞：
   连续 IR_DEBOUNCE 个周期都看到才置 on，连续 IR_DEBOUNCE 个周期
   都看不到才清 on，避免刚好在临界距离上来回抖。 */
static uint8_t IR_Update(IR_SideTypeDef *side, uint8_t raw)
{
    if (raw)
    {
        if (side->cnt < IR_DEBOUNCE) side->cnt++;
    }
    else
    {
        if (side->cnt > 0) side->cnt--;
    }

    side->on = (uint8_t)(side->cnt >= IR_DEBOUNCE);
    return side->on;
}

/* ---------------------------------------------------------------------------
 *  为了过链接必须留着的 reverse()
 *
 *  工程文件清单沿用了 TraceTest，里面带着四块灰阶驱动，它们都调用 reverse()，
 *  而 reverse() 定义在 APP\trace\trace.c，不在本工程里。本程序不用灰阶，
 *  这里只是补一个定义，避免 L6218E: Undefined symbol reverse。
 * ------------------------------------------------------------------------- */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}

/* ---------------------------------------------------------------------------
 *  三、电机下发
 * ------------------------------------------------------------------------- */

/* motor.c 里 MOTOR_Data[] 是全局的，但 motor.h 没声明。
   直接调 I2C_Write_Len() 才能拿到返回值（MOTOR_CONTROL_DATA_Predit() 会丢掉）。 */
extern int8_t MOTOR_Data[];

/* 总线恢复要用它们切 SDA 方向。 */
extern void I2C_SDA_OUT(void);
extern void I2C_SDA_IN(void);

/* 从机把 SDA 拉死不放时，补 9 个 SCL 脉冲 + 一个 STOP。 */
static void IR_I2C_Recover(void)
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

/* vx 取 -1/0/+1，vy 取 -1/0/+1。两个轴同时动时轮速减半，
   这样斜着走不会比直着走快一倍。 */
static void IR_Mix(int16_t vx, int16_t vy,
                   int16_t *m201, int16_t *m202,
                   int16_t *m203, int16_t *m204)
{
    int16_t amp = IR_SPEED;

    if ((vx != 0) && (vy != 0)) amp = (int16_t)(IR_SPEED / 2);

    *m201 = (int16_t)((vx + vy) * amp);
    *m202 = (int16_t)((vx - vy) * amp);
    *m203 = (int16_t)((vx + vy) * amp);
    *m204 = (int16_t)((vx - vy) * amp);
}

/* 返回 0 = 写成功，1 = 几次重试都失败。 */
static uint8_t IR_Send(int16_t m201, int16_t m202, int16_t m203, int16_t m204)
{
    uint8_t r;

    MOTOR_Data[0] = (int8_t)m203;
    MOTOR_Data[1] = (int8_t)m202;
    MOTOR_Data[2] = (int8_t)m204;
    MOTOR_Data[3] = (int8_t)m201;

    for (r = 0; r < IR_RETRY; r++)
    {
        if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, MOTOR_COUNT) == 0)
            return 0;
        IR_I2C_Recover();
    }
    return 1;
}

/* ---------------------------------------------------------------------------
 *  四、主循环
 * ------------------------------------------------------------------------- */

int main(void)
{
    uint8_t i;
    uint8_t fl, fr, bl, br, lu, ld, ru, rd;
    uint8_t front, back, left, right;
    uint8_t io_err_ticks = 0;
    uint16_t tick = 0;
    int16_t vx, vy;
    int16_t m201, m202, m203, m204;
    IR_SideTypeDef sFront = { 0, 0 };
    IR_SideTypeDef sBack  = { 0, 0 };
    IR_SideTypeDef sLeft  = { 0, 0 };
    IR_SideTypeDef sRight = { 0, 0 };

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);              /* delay_us / delay_ms 要用，必须最先做 */
    LED_Init();
    USART1_Init(19200);

    LED_RED   = IR_LED_OFF;
    LED_GREEN = IR_LED_OFF;

    MOTOR_Init();
    delay_ms(5);

    /* 上电先把四个轮子写停，并检查这次写是不是真的成功了。
       不检查的话，一旦这次写丢了，板子会保持上一次的转速。 */
    for (i = 0; i < 3; i++)
    {
        if (IR_Send(0, 0, 0, 0) == 0) break;
        delay_ms(50);
    }

    IR_SensorInit();

#if IR_DEBUG
    printf("\r\n== IRTest (4-direction obstacle avoidance) ==\r\n");
    printf("IRTest: speed=%d tick=%dms debounce=%d active_low=%d\r\n",
           (int)IR_SPEED, (int)IR_TICK_MS, (int)IR_DEBOUNCE, (int)IR_ACTIVE_LOW);
    printf("IRTest: FL=PD13 FR=PE12 BL=PB7 BR=PE4 LU=PD14 LD=PB8 RU=PE11 RD=PE3\r\n");
    printf("IRTest: front->back  back->front  left->right  right->left\r\n");
    printf("IRTest: start in %d ms\r\n", (int)IR_START_DELAY_MS);
#endif

    delay_ms(IR_START_DELAY_MS);

    for (;;)
    {
        /* ---------- 1. 读八路，按四侧去抖 ---------- */
        fl = (uint8_t)IR_FRONT_L;
        fr = (uint8_t)IR_FRONT_R;
        bl = (uint8_t)IR_BACK_L;
        br = (uint8_t)IR_BACK_R;
        lu = (uint8_t)IR_LEFT_U;
        ld = (uint8_t)IR_LEFT_D;
        ru = (uint8_t)IR_RIGHT_U;
        rd = (uint8_t)IR_RIGHT_D;

        front = IR_Update(&sFront, (uint8_t)(fl || fr));
        back  = IR_Update(&sBack,  (uint8_t)(bl || br));
        left  = IR_Update(&sLeft,  (uint8_t)(lu || ld));
        right = IR_Update(&sRight, (uint8_t)(ru || rd));

        /* ---------- 2. 算"往哪走" ----------
           前面有 -> vx = -1（后退）      后面有 -> vx = +1（前进）
           左面有 -> vy = -1（右移）      右面有 -> vy = +1（左移）
           前后都有 -> vx = 0；左右都有 -> vy = 0。 */
        vx = (int16_t)(IR_SIGN_VX * ((back ? 1 : 0) - (front ? 1 : 0)));
        vy = (int16_t)(IR_SIGN_VY * ((right ? 1 : 0) - (left ? 1 : 0)));

        /* ---------- 3. 混成四个轮速并下发 ---------- */
        IR_Mix(vx, vy, &m201, &m202, &m203, &m204);

        if (IR_Send(m201, m202, m203, m204) != 0)
            io_err_ticks = 40;      /* 故障灯至少亮 40 个周期（800ms） */

        /* ---------- 4. 指示灯 ---------- */
        if (io_err_ticks > 0)
        {
            io_err_ticks--;
            LED_RED   = ((tick / 2) & 1) ? IR_LED_ON : IR_LED_OFF;
            LED_GREEN = ((tick / 2) & 1) ? IR_LED_OFF : IR_LED_ON;
        }
        else if (front || back || left || right)
        {
            LED_RED   = IR_LED_ON;      /* 红灯常亮：正在避障 */
            LED_GREEN = IR_LED_OFF;
        }
        else
        {
            LED_RED   = IR_LED_OFF;
            LED_GREEN = IR_LED_ON;      /* 绿灯常亮：没有障碍，四轮停 */
        }

        /* ---------- 5. 串口 ---------- */
#if IR_DEBUG
        if ((tick % IR_PRINT_EVERY) == 0)
        {
            printf("IR: FL%d FR%d BL%d BR%d LU%d LD%d RU%d RD%d | "
                   "F%d B%d L%d R%d | vx %2d vy %2d | "
                   "m201 %3d m202 %3d m203 %3d m204 %3d%s\r\n",
                   (int)fl, (int)fr, (int)bl, (int)br,
                   (int)lu, (int)ld, (int)ru, (int)rd,
                   (int)front, (int)back, (int)left, (int)right,
                   (int)vx, (int)vy,
                   (int)m201, (int)m202, (int)m203, (int)m204,
                   (io_err_ticks > 0) ? " | ERR" : "");
        }
#endif

        tick++;
        delay_ms(IR_TICK_MS);
    }
}
