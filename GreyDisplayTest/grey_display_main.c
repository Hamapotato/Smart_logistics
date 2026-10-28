/* ============================================================================
 * GreyDisplayTest —— 八路灰度黑线通道数码管显示
 *
 * 默认只测试前向模块。
 * 数码管按模块方向从左到右显示 1..8 中被识别为黑线的路号。
 * 最多显示 4 路；0 路、超过 4 路或读取失败时清空数码管。
 *
 * 本工程不初始化任何电机，也不驱动底盘。
 * ========================================================================== */

#include "main.h"

#define GREY_DIR_FRONT               0U
#define GREY_DIR_BACK                1U
#define GREY_DIR_LEFT                2U
#define GREY_DIR_RIGHT               3U

/* 当前先测试前向；以后只改这一行即可切换其他方向。 */
#define GREY_TEST_DIR                GREY_DIR_FRONT

#if   (GREY_TEST_DIR == GREY_DIR_FRONT)
  #define GREY_DIR_NAME              "FRONT"
#elif (GREY_TEST_DIR == GREY_DIR_BACK)
  #define GREY_DIR_NAME              "BACK"
#elif (GREY_TEST_DIR == GREY_DIR_LEFT)
  #define GREY_DIR_NAME              "LEFT"
#elif (GREY_TEST_DIR == GREY_DIR_RIGHT)
  #define GREY_DIR_NAME              "RIGHT"
#else
  #error "Invalid GREY_TEST_DIR"
#endif

#define GREY_TICK_MS                 20U
#define GREY_WARMUP_MS               20000U
#define GREY_YB_ADDR_DEF             0x12U
#define GREY_YB_ADDR8                ((uint8_t)(GREY_YB_ADDR_DEF << 1))
#define GREY_YB_DATA_REG             0x30U
#define GREY_YB_CALIB_REG            0x01U

extern sw_i2c_interface_t f_i2c_interface;
extern sw_i2c_interface_t b_i2c_interface;
extern sw_i2c_interface_t l_i2c_interface;
extern sw_i2c_interface_t r_i2c_interface;

extern int8_t f_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t b_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t l_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t r_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);

extern void f_sw_i2c_init(void);
extern void b_sw_i2c_init(void);
extern void l_sw_i2c_init(void);
extern void r_sw_i2c_init(void);

uint8_t reverse(uint8_t x);

volatile uint32_t GreyDebugLoops;
volatile uint8_t  GreyDebugRaw;
volatile uint8_t  GreyDebugBits;
volatile uint8_t  GreyDebugReadOk;

static uint8_t Grey_Bits(uint8_t raw)
{
    uint8_t v;

    /*
     * The installed module reports one bit offset from the reference
     * bit order. This is the mapping validated in StraightTest:
     * reverse to physical order, rotate right by one, then invert
     * because the module reports black as 0 and white as 1.
     */
    v = reverse(raw);
    v = (uint8_t)((v >> 1) | (v << 7));
    return (uint8_t)((~v) & GREY6_ACTIVE_MASK);
}

static void Grey_SensorInit(void)
{
#if   (GREY_TEST_DIR == GREY_DIR_FRONT)
    f_sw_i2c_init();
#elif (GREY_TEST_DIR == GREY_DIR_BACK)
    b_sw_i2c_init();
#elif (GREY_TEST_DIR == GREY_DIR_LEFT)
    l_sw_i2c_init();
#else
    r_sw_i2c_init();
#endif
}

static uint8_t Grey_ReadRaw(uint8_t *raw)
{
#if   (GREY_TEST_DIR == GREY_DIR_FRONT)
    return (f_sw_i2c_mem_read(&f_i2c_interface, GREY_YB_ADDR8,
                              GREY_YB_DATA_REG, raw, 1U) == 0)
         ? 1U : 0U;
#elif (GREY_TEST_DIR == GREY_DIR_BACK)
    return (b_sw_i2c_mem_read(&b_i2c_interface, GREY_YB_ADDR8,
                              GREY_YB_DATA_REG, raw, 1U) == 0)
         ? 1U : 0U;
#elif (GREY_TEST_DIR == GREY_DIR_LEFT)
    return (l_sw_i2c_mem_read(&l_i2c_interface, GREY_YB_ADDR8,
                              GREY_YB_DATA_REG, raw, 1U) == 0)
         ? 1U : 0U;
#else
    return (r_sw_i2c_mem_read(&r_i2c_interface, GREY_YB_ADDR8,
                              GREY_YB_DATA_REG, raw, 1U) == 0)
         ? 1U : 0U;
#endif
}

static void Grey_ShowBlackChannels(uint8_t bits, uint8_t read_ok)
{
    unsigned char digits[4] = { 10U, 10U, 10U, 10U };
    uint8_t count = 0U;
    uint8_t pos;
    uint8_t ch;

    if (read_ok)
    {
        for (ch = 1U; ch < 7U; ch++)
            if (bits & (uint8_t)(1U << ch)) count++;
    }

    if ((count > 0U) && (count <= 4U))
    {
        pos = (uint8_t)(4U - count);
        for (ch = 1U; ch < 7U; ch++)
        {
            if (bits & (uint8_t)(1U << ch))
                digits[pos++] = (unsigned char)(ch + 1U);
        }
    }

    TUBES_DisplayDigits(digits);
}

int main(void)
{
    uint8_t raw;
    uint8_t bits;
    uint8_t read_ok;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168U);

    TUBES_Driver_Init();
    TUBES_SetBrightness(6U);
    TUBES_Clear();

    Grey_SensorInit();
    delay_ms(GREY_WARMUP_MS);

    (void)GREY_DIR_NAME;

    for (;;)
    {
        raw = 0U;
        read_ok = Grey_ReadRaw(&raw);
        bits = read_ok ? Grey_Bits(raw) : 0U;

        GreyDebugRaw = raw;
        GreyDebugBits = bits;
        GreyDebugReadOk = read_ok;
        GreyDebugLoops++;

        Grey_ShowBlackChannels(bits, read_ok);
        delay_ms(GREY_TICK_MS);
    }
}

/* 工程文件清单包含灰度驱动，它们链接时需要 reverse()。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAAU) >> 1) | ((x & 0x55U) << 1));
    x = (uint8_t)(((x & 0xCCU) >> 2) | ((x & 0x33U) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}
