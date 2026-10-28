/* Four-direction straight-line test using four YB-MUX04-2.0 sensors. */
#include "main.h"

#define TEST_DIR_FRONT              0
#define TEST_DIR_BACK               1
#define TEST_DIR_LEFT               2
#define TEST_DIR_RIGHT              3

/* Change only this line to select the direction under test. */
#define TEST_DIR                    TEST_DIR_BACK

/* Independent correction switches. */
#define TEST_TRANSLATE_ENABLE       0
#define TEST_YAW_ENABLE             1

#define TEST_SPEED                  10
#define TEST_TICK_MS                20
#define TEST_START_DELAY_MS         1000

#define TEST_TRANSLATE_K            8
#define TEST_TRANSLATE_MAX          2
#define TEST_YAW_K                  8
#define TEST_YAW_MAX                1
#define TEST_DEADBAND               2

/* Per-direction calibration signs. */
#define TEST_TRANSLATE_SIGN_FRONT   (+1)
#define TEST_TRANSLATE_SIGN_BACK    (+1)
#define TEST_TRANSLATE_SIGN_LEFT    (+1)
#define TEST_TRANSLATE_SIGN_RIGHT   (+1)

#define TEST_YAW_SIGN_FRONT         (+1)
#define TEST_YAW_SIGN_BACK          (-1)  /* backward */
#define TEST_YAW_SIGN_LEFT          (-1)
#define TEST_YAW_SIGN_RIGHT         (-1)

#if   (TEST_DIR == TEST_DIR_FRONT)
  #define TEST_USE_FRONT_BACK       1
  #define TEST_TRANSLATE_SIGN       TEST_TRANSLATE_SIGN_FRONT
  #define TEST_YAW_SIGN             TEST_YAW_SIGN_FRONT
  #define TEST_DIR_NAME             "FRONT"
#elif (TEST_DIR == TEST_DIR_BACK)
  #define TEST_USE_FRONT_BACK       1
  #define TEST_TRANSLATE_SIGN       TEST_TRANSLATE_SIGN_BACK
  #define TEST_YAW_SIGN             TEST_YAW_SIGN_BACK
  #define TEST_DIR_NAME             "BACK"
#elif (TEST_DIR == TEST_DIR_LEFT)
  #define TEST_USE_FRONT_BACK       0
  #define TEST_TRANSLATE_SIGN       TEST_TRANSLATE_SIGN_LEFT
  #define TEST_YAW_SIGN             TEST_YAW_SIGN_LEFT
  #define TEST_DIR_NAME             "LEFT"
#elif (TEST_DIR == TEST_DIR_RIGHT)
  #define TEST_USE_FRONT_BACK       0
  #define TEST_TRANSLATE_SIGN       TEST_TRANSLATE_SIGN_RIGHT
  #define TEST_YAW_SIGN             TEST_YAW_SIGN_RIGHT
  #define TEST_DIR_NAME             "RIGHT"
#else
  #error "TEST_DIR must be TEST_DIR_FRONT/BACK/LEFT/RIGHT"
#endif

#define TEST_DEBUG                  1
#define TEST_PRINT_EVERY            25
#define TEST_RETRY                  3
#define TEST_LED_ON                 0
#define TEST_LED_OFF                1

#define TEST_YB_ADDR_DEF            0x12
#define TEST_YB_ADDR8               ((uint8_t)(TEST_YB_ADDR_DEF << 1))
#define TEST_YB_STATE_CMD           0x01

extern sw_i2c_interface_t f_i2c_interface;
extern sw_i2c_interface_t b_i2c_interface;
extern sw_i2c_interface_t l_i2c_interface;
extern sw_i2c_interface_t r_i2c_interface;

extern int8_t f_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t,
                                uint8_t, uint8_t *, uint8_t);
extern int8_t b_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t,
                                uint8_t, uint8_t *, uint8_t);
extern int8_t l_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t,
                                uint8_t, uint8_t *, uint8_t);
extern int8_t r_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t,
                                uint8_t, uint8_t *, uint8_t);

extern void f_sw_i2c_init(void);
extern void b_sw_i2c_init(void);
extern void l_sw_i2c_init(void);
extern void r_sw_i2c_init(void);

extern int8_t MOTOR_Data[];
extern void I2C_SDA_OUT(void);
extern void I2C_SDA_IN(void);

uint8_t reverse(uint8_t x);

static void Test_Recover(void)
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
}

static uint8_t Test_WriteBody(const int16_t body[4])
{
    uint8_t retry;

    MOTOR_Data[0] = (int8_t)body[2];
    MOTOR_Data[1] = (int8_t)body[1];
    MOTOR_Data[2] = (int8_t)body[3];
    MOTOR_Data[3] = (int8_t)body[0];

    for (retry = 0; retry < TEST_RETRY; retry++)
    {
        if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, 4) == 0)
            return 0;
        Test_Recover();
    }
    return 1;
}

static void Test_Stop(void)
{
    int16_t zero[4] = { 0, 0, 0, 0 };
    Test_WriteBody(zero);
}

static uint8_t Test_Mix(int16_t vx, int16_t vy, int16_t w)
{
    int16_t body[4];

    body[0] = (int16_t)(vx + vy);
    body[1] = (int16_t)(vx - vy);
    body[2] = (int16_t)(vx + vy);
    body[3] = (int16_t)(vx - vy);

    body[0] = (int16_t)(body[0] + w);
    body[1] = (int16_t)(body[1] + w);
    body[2] = (int16_t)(body[2] - w);
    body[3] = (int16_t)(body[3] - w);
    return Test_WriteBody(body);
}

static uint8_t Test_YBBits(uint8_t raw)
{
    uint8_t v = reverse(raw);
    v = (uint8_t)((v >> 1) | (v << 7));
    return (uint8_t)(~v);
}

static uint8_t Test_Valid(uint8_t bits)
{
    uint8_t count = 0;
    uint8_t transitions = 0;
    uint8_t i;
    uint8_t v = bits;

    while (v) {
        count = (uint8_t)(count + (v & 1U));
        v = (uint8_t)(v >> 1);
    }
    if ((count == 0) || (count == 8)) return 0;

    for (i = 1; i < 8; i++)
    {
        if (((bits >> i) & 1U) != ((bits >> (i - 1)) & 1U))
            transitions++;
    }
    return (transitions <= 2) ? 1U : 0U;
}

static uint8_t Test_EdgeMask(uint8_t bits)
{
    uint8_t middle = (uint8_t)(bits & 0x18U);

    if (middle != 0x18U) return bits;
    if ((bits & 0x07U) == 0x07U) return 0;
    if ((bits & 0xE0U) == 0xE0U) return 0;
    return bits;
}

static int16_t Test_Weighted(uint8_t bits, const int8_t weight[8])
{
    int16_t sum = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
        if (bits & (uint8_t)(1U << i))
            sum = (int16_t)(sum + weight[i]);
    return sum;
}

static int16_t Test_Translation(int16_t error)
{
    int16_t v;

    if ((error >= -TEST_DEADBAND) && (error <= TEST_DEADBAND))
        return 0;

    v = (int16_t)((error * TEST_TRANSLATE_K) / 8);
    if (v > TEST_TRANSLATE_MAX) v = TEST_TRANSLATE_MAX;
    if (v < -TEST_TRANSLATE_MAX) v = -TEST_TRANSLATE_MAX;
    return (int16_t)(TEST_TRANSLATE_SIGN * v);
}

static int16_t Test_Yaw(int16_t error)
{
    int16_t v;

    if ((error >= -TEST_DEADBAND) && (error <= TEST_DEADBAND))
        return 0;

    v = (int16_t)((error * TEST_YAW_K) / 8);
    if (v > TEST_YAW_MAX) v = TEST_YAW_MAX;
    if (v < -TEST_YAW_MAX) v = -TEST_YAW_MAX;
    return (int16_t)(TEST_YAW_SIGN * v);
}

static uint8_t Test_ReadPair(uint8_t *a_raw, uint8_t *b_raw)
{
#if TEST_USE_FRONT_BACK
    if (f_sw_i2c_mem_read(&f_i2c_interface, TEST_YB_ADDR8,
                          TEST_YB_STATE_CMD, a_raw, 1) != 0) return 0;
    if (b_sw_i2c_mem_read(&b_i2c_interface, TEST_YB_ADDR8,
                          TEST_YB_STATE_CMD, b_raw, 1) != 0) return 0;
#else
    if (l_sw_i2c_mem_read(&l_i2c_interface, TEST_YB_ADDR8,
                          TEST_YB_STATE_CMD, a_raw, 1) != 0) return 0;
    if (r_sw_i2c_mem_read(&r_i2c_interface, TEST_YB_ADDR8,
                          TEST_YB_STATE_CMD, b_raw, 1) != 0) return 0;
#endif
    return 1;
}

static uint8_t Test_PairBits(uint8_t a_raw, uint8_t b_raw,
                             uint8_t *a_bits, uint8_t *b_bits)
{
    *a_bits = Test_YBBits(a_raw);
    *b_bits = Test_YBBits(b_raw);

#if !TEST_USE_FRONT_BACK
    /* Left sensor is 8..1 front-to-back; normalize it to 1..8. */
    *a_bits = reverse(*a_bits);
#endif

    if (!Test_Valid(*a_bits) && !Test_Valid(*b_bits)) return 0;
    return 1;
}

int main(void)
{
    uint8_t a_raw = 0;
    uint8_t b_raw = 0;
    uint8_t a_bits = 0;
    uint8_t b_bits = 0;
    uint8_t a_ctrl = 0;
    uint8_t b_ctrl = 0;
    uint8_t a_ok;
    uint8_t b_ok;
    uint16_t tick = 0;
    int16_t translation;
    int16_t yaw_error;
    int16_t vx;
    int16_t vy;
    int16_t w;
    static const int8_t POS_W[8] = { +8, +5, +2, +1, -1, -2, -5, -8 };
    static const int8_t YAW_W[8] = { +8, +5, +2, +1, -1, -2, -5, -8 };

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    USART1_Init(19200);

    LED_RED   = TEST_LED_OFF;
    LED_GREEN = TEST_LED_OFF;

    MOTOR_Init();
    Test_Stop();
    delay_ms(100);

#if TEST_USE_FRONT_BACK
    f_sw_i2c_init();
    b_sw_i2c_init();
#else
    l_sw_i2c_init();
    r_sw_i2c_init();
#endif

#if TEST_DEBUG
    printf("\r\n== FourDirStraightTest: %s ==\r\n", TEST_DIR_NAME);
    printf("speed=%d translate=%d yaw=%d tick=%dms\r\n",
           (int)TEST_SPEED, (int)TEST_TRANSLATE_ENABLE,
           (int)TEST_YAW_ENABLE, (int)TEST_TICK_MS);
#endif

    delay_ms(TEST_START_DELAY_MS);

    for (;;)
    {
        if (Test_ReadPair(&a_raw, &b_raw) == 0)
        {
            Test_Stop();
            LED_RED = TEST_LED_ON;
            LED_GREEN = TEST_LED_OFF;
            delay_ms(TEST_TICK_MS);
            continue;
        }

        if (Test_PairBits(a_raw, b_raw, &a_bits, &b_bits) == 0)
        {
            Test_Stop();
            LED_RED = TEST_LED_ON;
            LED_GREEN = TEST_LED_OFF;
            delay_ms(TEST_TICK_MS);
            continue;
        }

        a_ok = Test_Valid(a_bits);
        b_ok = Test_Valid(b_bits);
        a_ctrl = a_ok ? Test_EdgeMask(a_bits) : 0;
        b_ctrl = b_ok ? Test_EdgeMask(b_bits) : 0;

        translation = (int16_t)((a_ok ? Test_Weighted(a_ctrl, POS_W) : 0)
                              + (b_ok ? Test_Weighted(b_ctrl, POS_W) : 0));
        yaw_error = (a_ok && b_ok)
                  ? (int16_t)(Test_Weighted(a_ctrl, YAW_W)
                            - Test_Weighted(b_ctrl, YAW_W))
                  : 0;

#if !TEST_TRANSLATE_ENABLE
        translation = 0;
#endif
#if !TEST_YAW_ENABLE
        yaw_error = 0;
#endif

        w = Test_Yaw(yaw_error);

#if   (TEST_DIR == TEST_DIR_FRONT)
        vx = TEST_SPEED;
        vy = Test_Translation(translation);
#elif (TEST_DIR == TEST_DIR_BACK)
        vx = -TEST_SPEED;
        vy = Test_Translation(translation);
#elif (TEST_DIR == TEST_DIR_LEFT)
        vx = Test_Translation(translation);
        vy = TEST_SPEED;
#else
        vx = Test_Translation(translation);
        vy = -TEST_SPEED;
#endif

        if (Test_Mix(vx, vy, w))
        {
            Test_Stop();
            LED_RED = TEST_LED_ON;
            LED_GREEN = TEST_LED_OFF;
        }
        else
        {
            LED_RED = TEST_LED_OFF;
            LED_GREEN = TEST_LED_ON;
        }

#if TEST_DEBUG
        if ((tick % TEST_PRINT_EVERY) == 0)
        {
            printf("FourDir[%s]: A=%02X->%02X B=%02X->%02X trans=%d yaw=%d vx=%d vy=%d w=%d\r\n",
                   TEST_DIR_NAME,
                   (unsigned int)a_raw, (unsigned int)a_bits,
                   (unsigned int)b_raw, (unsigned int)b_bits,
                   (int)translation, (int)yaw_error,
                   (int)vx, (int)vy, (int)w);
        }
#endif

        tick++;
        delay_ms(TEST_TICK_MS);
    }
}

uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}
