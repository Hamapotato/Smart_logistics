/* ============================================================================
 * StraightTest —— 前后双灰阶直线循迹测试
 *
 * 【硬件范围】
 *   前灰阶：PB13=SCL，PB14=SDA
 *   后灰阶：PD1 =SCL，PD2 =SDA
 *   前地址 0x12（命令 0x01），后地址 0x4F
 *
 *   本测试只初始化并使用前、后两块灰阶。
 *   左、右两块灰阶完全不初始化、不读取。
 *
 * 【有效数据规则】
 *   前后灰阶只使用 L2-L7；六路全白 0x00 或全黑 0x7E 时判为不可信，
 *   本周期忽略该传感器。两块都不可信时停车，不驱动电机。
 *
 * 【位姿计算】
 *   前灰阶：1~4 为左，5~8 为右
 *   后灰阶：1~4 为右，5~8 为左
 *
 *   平移权重：
 *       前：+4,+3,+2,0,0,-2,-3,-4
 *       后：-4,-3,-2,0,0,+2,+3,+4
 *
 *   转弯权重：
 *       前/后：+4,+3,+2,+1,-1,-2,-3,-4
 *
 *   E_LAT = 前平移权重和 + 后平移权重和
 *   E_YAW = 前转弯权重和 + 后转弯权重和
 *
 *   例如：
 *       前1/后8：平移权重同号，转弯权重抵消 -> 平移修正
 *       前1/后1：转弯权重同号，平移权重抵消 -> 转弯修正
 *
 * 【底盘】
 *   参考 MoveTest，直接向 0x33 写四轮速度，不使用编码器速度闭环。
 * ========================================================================== */

#include "main.h"

/* ---------------------------------------------------------------------------
 *  一、参数
 * ------------------------------------------------------------------------- */

#define STRAIGHT_CRUISE             10
#define STRAIGHT_TICK_MS            20
#define STRAIGHT_START_DELAY_MS     1000

/* 平移纠偏量 = E_LAT * K / 8。 */
#define STRAIGHT_LAT_K              8
#define STRAIGHT_LAT_MAX            2
#define STRAIGHT_LAT_SIGN           (+1)

/* 转弯纠偏量 = E_YAW * K / 8。 */
#define STRAIGHT_YAW_K              8
#define STRAIGHT_YAW_MAX            1
#define STRAIGHT_YAW_SIGN           (1)

/* 1 = 原始 bit 为黑，0 = 原始 bit 取反后为黑。 */

/* 2026-10-27：前灰阶更换后数字量极性相反，整字节取反。
   只影响前灰阶计算，后灰阶保持原样。 */

#define STRAIGHT_DEADBAND           2
#define STRAIGHT_RETRY              3
#define STRAIGHT_DEBUG              1
#define STRAIGHT_PRINT_EVERY        25
#define STRAIGHT_TUBE_EVERY         5
#define STRAIGHT_GW_BLACK_LEVEL     1

#define STRAIGHT_LED_ON             0
#define STRAIGHT_LED_OFF            1

/* 前/后灰阶底层接口。 */
extern sw_i2c_interface_t f_i2c_interface;
extern sw_i2c_interface_t b_i2c_interface;
extern int8_t f_sw_i2c_read_byte(sw_i2c_interface_t *,
                                 uint8_t, uint8_t *);
extern int8_t f_sw_i2c_read_byte(sw_i2c_interface_t *,
                                 uint8_t, uint8_t *);
extern int8_t b_sw_i2c_read_byte(sw_i2c_interface_t *,
                                 uint8_t, uint8_t *);

#define STRAIGHT_GW_ADDR8 ((uint8_t)(GW_GRAY_ADDR_DEF << 1))

uint8_t reverse(uint8_t x);

extern int8_t MOTOR_Data[];
extern void I2C_SDA_OUT(void);
extern void I2C_SDA_IN(void);

/* ---------------------------------------------------------------------------
 *  二、四轮速度下发
 * ------------------------------------------------------------------------- */

static void Straight_I2C_Recover(void)
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

/* body 顺序：201=右前，202=右后，203=左后，204=左前。 */
static uint8_t Straight_WriteBody(const int16_t body[4])
{
    uint8_t retry;

    MOTOR_Data[0] = (int8_t)body[2]; /* CH1=203 */
    MOTOR_Data[1] = (int8_t)body[1]; /* CH2=202 */
    MOTOR_Data[2] = (int8_t)body[3]; /* CH3=204 */
    MOTOR_Data[3] = (int8_t)body[0]; /* CH4=201 */

    for (retry = 0; retry < STRAIGHT_RETRY; retry++)
    {
        if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, 4) == 0)
            return 0;
        Straight_I2C_Recover();
    }
    return 1;
}

static void Straight_Stop(void)
{
    int16_t zero[4] = { 0, 0, 0, 0 };
    Straight_WriteBody(zero);
}

/* vx>0 前进，vy>0 左移，w 为原地旋转。 */
static uint8_t Straight_Mix(int16_t vx, int16_t vy, int16_t w)
{
    int16_t body[4];

    body[0] = (int16_t)(vx + vy); /* 201 */
    body[1] = (int16_t)(vx - vy); /* 202 */
    body[2] = (int16_t)(vx + vy); /* 203 */
    body[3] = (int16_t)(vx - vy); /* 204 */

    /* 纯旋转分量：四轮变化量绝对值都为 |w|，旋转中心保持在车体中心。 */
    body[0] = (int16_t)(body[0] + w);
    body[1] = (int16_t)(body[1] + w);
    body[2] = (int16_t)(body[2] - w);
    body[3] = (int16_t)(body[3] - w);
    return Straight_WriteBody(body);
}

/* ---------------------------------------------------------------------------
 *  三、前后灰阶
 * ------------------------------------------------------------------------- */

static uint8_t Straight_ReadFront(uint8_t *raw)
{
    return (f_sw_i2c_read_byte(&f_i2c_interface, STRAIGHT_GW_ADDR8, raw) == 0)
         ? 1U : 0U;
}

static uint8_t Straight_GWBits(uint8_t raw)
{
#if STRAIGHT_GW_BLACK_LEVEL
    return (uint8_t)(raw & GREY6_ACTIVE_MASK);
#else
    return (uint8_t)((~raw) & GREY6_ACTIVE_MASK);
#endif
}

static uint8_t Straight_FrontBits(uint8_t raw)
{
    return Straight_GWBits(raw);
}

static uint8_t Straight_BackBits(uint8_t raw)
{
    return Straight_GWBits(raw);
}

/* A wide edge pattern is ignored only when the middle 45 is also black:
   123+45 -> no correction, or 678+45 -> no correction. */
static uint8_t Straight_EdgeMask(uint8_t bits)
{
    return (uint8_t)(bits & GREY6_ACTIVE_MASK);
}

/* Show black front channels, ch1=leftmost through ch8=rightmost.
   More than four black channels, no black channel, or read failure clears the display. */
static void Straight_ShowBlackChannels(uint8_t bits, uint8_t read_ok)
{
    unsigned char digits[4] = {10, 10, 10, 10};
    uint8_t ch;
    uint8_t count = 0;
    uint8_t pos;

    if (read_ok) {
        for (ch = 0; ch < 8; ch++) {
            if (bits & (uint8_t)(1u << ch)) count++;
        }
    }

    if ((count > 0) && (count <= 4)) {
        pos = (uint8_t)(4 - count);
        for (ch = 0; ch < 8; ch++) {
            if (bits & (uint8_t)(1u << ch)) {
                digits[pos++] = (unsigned char)(ch + 1);
            }
        }
    }

    TUBES_DisplayDigits(digits);
}

static uint8_t Straight_ReadBack(uint8_t *raw)
{
    return (b_sw_i2c_read_byte(&b_i2c_interface, STRAIGHT_GW_ADDR8, raw) == 0)
         ? 1U : 0U;
}

/* 用户规则：八路全 0 或全 1 都认为该传感器本周期不可信。 */
static uint8_t Straight_Valid(uint8_t raw)
{
    return ((raw != 0x00U) && (raw != GREY6_ACTIVE_MASK)) ? 1U : 0U;
}

static int16_t Straight_Weighted(uint8_t raw, const int8_t w[8])
{
    int16_t sum = 0;
    uint8_t bits = raw;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (bits & (uint8_t)(1U << i)) sum = (int16_t)(sum + w[i]);
    }
    return sum;
}

/* 平移权重：1、8 外侧大，4、5 中心为 0。
   前后统一按 1~4 左、5~8 右解释。 */
static int16_t Straight_Lat(uint8_t raw, uint8_t back)
{
    static const int8_t FRONT_W[8] = { 0, +8, +5, +1, -1, -5, -8, 0 };
    static const int8_t BACK_W[8]  = { 0, +8, +5, +1, -1, -5, -8, 0 };
    return Straight_Weighted(raw, back ? BACK_W : FRONT_W);
}

/* 转弯权重：前1~4为左、5~8为右；后1~4为右、5~8为左。
   组合后前1/后1、前8/后8都得到同一个方向的转角误差。 */
static int16_t Straight_Yaw(uint8_t raw)
{
    static const int8_t YAW_W[8] = { 0, +8, +5, +1, -1, -5, -8, 0 };
    return Straight_Weighted(raw, YAW_W);
}

static int16_t Straight_Clamp(int16_t value, int16_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int16_t Straight_Channel(int16_t e, int16_t k,
                                int16_t limit, int8_t sign)
{
    int16_t v = (int16_t)((e * k) / 8);
    if (v > limit) v = limit;
    if (v < -limit) v = -limit;
    return (int16_t)(sign * v);
}

/* ---------------------------------------------------------------------------
 *  四、主程序
 * ------------------------------------------------------------------------- */

int main(void)
{
    uint8_t front_raw = 0;
    uint8_t front_bits = 0;
    uint8_t front_ctrl_bits = 0;
    uint8_t back_raw = 0;
    uint8_t back_bits = 0;
    uint8_t back_ctrl_bits = 0;
    uint8_t front_ok;
    uint8_t back_ok;
    int16_t front_e = 0;
    int16_t back_e  = 0;
    int16_t e_lat   = 0;
    int16_t e_yaw   = 0;
    int16_t vy      = 0;
    int16_t w       = 0;
    uint16_t tick = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    USART1_Init(19200);
    TUBES_Driver_Init();
    TUBES_SetBrightness(6);
    TUBES_Clear();

    LED_RED   = STRAIGHT_LED_OFF;
    LED_GREEN = STRAIGHT_LED_OFF;

    MOTOR_Init();
    Straight_Stop();
    delay_ms(100);

    F_GW_digital_Init();
    B_GW_digital_Init();
    delay_ms(100);

    /* 上电等待，同时确认至少一块传感器有可信数据。 */
#if STRAIGHT_DEBUG
    printf("\r\n== StraightTest: front + back grey only ==\r\n");
    printf("StraightTest: cruise=%d latK=%d/%d yawK=%d/%d tick=%dms\r\n",
           (int)STRAIGHT_CRUISE,
           (int)STRAIGHT_LAT_K, (int)STRAIGHT_LAT_MAX,
           (int)STRAIGHT_YAW_K, (int)STRAIGHT_YAW_MAX,
           (int)STRAIGHT_TICK_MS);
    printf("GWStraightTest: front/back GW 8ch @0x4F, black_level=%d\r\n",
           (int)STRAIGHT_GW_BLACK_LEVEL);
#endif

    delay_ms(STRAIGHT_START_DELAY_MS);

    for (;;)
    {
        front_ok = Straight_ReadFront(&front_raw);
        if (front_ok) {
            front_bits = Straight_FrontBits(front_raw);
            front_ok = Straight_Valid(front_bits);
        } else {
            front_bits = 0;
        }

        back_ok = Straight_ReadBack(&back_raw);
        if (back_ok) {
            back_bits = Straight_BackBits(back_raw);
            back_ok = Straight_Valid(back_bits);
        } else {
            back_bits = 0;
        }

        front_ctrl_bits = front_ok ? Straight_EdgeMask(front_bits) : 0;
        back_ctrl_bits  = back_ok  ? Straight_EdgeMask(back_bits)  : 0;

        if ((tick % STRAIGHT_TUBE_EVERY) == 0)
            Straight_ShowBlackChannels(front_bits, front_ok);

        if ((!front_ok) && (!back_ok))
        {
            Straight_Stop();
            LED_RED   = ((tick / 2) & 1) ? STRAIGHT_LED_OFF : STRAIGHT_LED_ON;
            LED_GREEN = STRAIGHT_LED_OFF;
        }
        else
        {
            front_e = front_ok ? Straight_Lat(front_ctrl_bits, 0) : 0;
            back_e  = back_ok  ? Straight_Lat(back_ctrl_bits, 1)  : 0;

            if (front_ok && back_ok)
            {
                /* 每个探头权重先分别累加，再把前后两路相加。 */
                e_lat = (int16_t)(front_e + back_e);
                e_yaw = (int16_t)(Straight_Yaw(front_ctrl_bits)
                                + Straight_Yaw(back_ctrl_bits));
            }
            else if (front_ok)
            {
                /* 只有前灰阶时只能做平移，不计算方向偏差。 */
                e_lat = front_e;
                e_yaw = 0;
            }
            else
            {
                /* 只有后灰阶时同样只做平移。 */
                e_lat = back_e;
                e_yaw = 0;
            }

            if ((e_lat >= -STRAIGHT_DEADBAND)
                && (e_lat <= STRAIGHT_DEADBAND))
                e_lat = 0;
            if ((e_yaw >= -STRAIGHT_DEADBAND)
                && (e_yaw <= STRAIGHT_DEADBAND))
                e_yaw = 0;

            vy = Straight_Channel(e_lat, STRAIGHT_LAT_K,
                                  STRAIGHT_LAT_MAX, STRAIGHT_LAT_SIGN);
            w  = Straight_Channel(e_yaw, STRAIGHT_YAW_K,
                                  STRAIGHT_YAW_MAX, STRAIGHT_YAW_SIGN);

            if (Straight_Mix(STRAIGHT_CRUISE, vy, w) != 0)
            {
                Straight_Stop();
                LED_RED   = ((tick / 2) & 1) ? STRAIGHT_LED_OFF : STRAIGHT_LED_ON;
                LED_GREEN = STRAIGHT_LED_OFF;
            }
            else
            {
                LED_RED   = STRAIGHT_LED_OFF;
                LED_GREEN = STRAIGHT_LED_ON;
            }
        }

#if STRAIGHT_DEBUG
        if ((tick % STRAIGHT_PRINT_EVERY) == 0)
        {
            printf("Straight: F=%02X->%02X C=%02X(%s) B=%02X->%02X C=%02X(%s) EF=%d EB=%d lat=%d yaw=%d | vy=%d w=%d\r\n",
                   (unsigned int)front_raw, (unsigned int)front_bits,
                   (unsigned int)front_ctrl_bits, front_ok ? "OK" : "--",
                   (unsigned int)back_raw, (unsigned int)back_bits,
                   (unsigned int)back_ctrl_bits, back_ok ? "OK" : "--",
                   (int)(front_ok ? front_e : 0),
                   (int)(back_ok ? back_e : 0),
                   (int)e_lat, (int)e_yaw, (int)vy, (int)w);
        }
#endif

        tick++;
        delay_ms(STRAIGHT_TICK_MS);
    }
}

/* 工程清单包含四块灰阶驱动，它们链接时需要 reverse()。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}
