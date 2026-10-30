/* ============================================================================
 * ServoTest —— PE5 单舵机测试
 *
 * PE5：TIM9_CH1
 * 舵机：60 kg·cm，270°，500~2500 us
 * PWM：50 Hz，20 ms 周期
 *
 * 单舵机三档：
 *   PE5  500 us = 投递/释放
 *   PE5 1500 us = 中间
 *   PE5 2500 us = 物品持有
 *
 * SERVO_INSTALL_MODE = 1：
 *   PE5 固定 2500 us，程序保持不动，用于安装舵盘。
 * ========================================================================== */

#include "main.h"

/* 1 = 舵盘安装模式；装好舵盘后改为 0，重新编译进入循环测试。 */
#define SERVO_INSTALL_MODE           0

#define SERVO_MIN_US                 500U
#define SERVO_MID_US                 800U
#define SERVO_MAX_US                 1300U

#define SERVO_PE5_DELIVER_US         SERVO_MIN_US
#define SERVO_PE5_HOLD_US            SERVO_MAX_US

#define SERVO_START_DELAY_MS         2000U
#define SERVO_HOLD_MS                1500U
#define SERVO_DEBUG                  1

#define SERVO_LED_ON                 0
#define SERVO_LED_OFF                1

#define SERVO_STATE_DELIVER          0U
#define SERVO_STATE_MID              1U
#define SERVO_STATE_HOLD             2U

#define SERVO_TEST_COUNT             4U

static const uint8_t Servo_TestStates[SERVO_TEST_COUNT] =
{
    SERVO_STATE_DELIVER,
    SERVO_STATE_MID,
    SERVO_STATE_HOLD,
    SERVO_STATE_MID
};

static uint16_t Servo_Clamp(uint16_t pulse)
{
    if (pulse < SERVO_MIN_US) return SERVO_MIN_US;
    if (pulse > SERVO_MAX_US) return SERVO_MAX_US;
    return pulse;
}

static void Servo_SetPulse(uint16_t pe5_us)
{
    TIM_SetCompare1(TIM9, Servo_Clamp(pe5_us));
}

static void Servo_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &gpio);

    TIM_TimeBaseStructInit(&time_base);
    time_base.TIM_Prescaler     = 168U - 1U;
    time_base.TIM_Period        = 20000U - 1U;
    time_base.TIM_CounterMode   = TIM_CounterMode_Up;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9, &time_base);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = SERVO_MID_US;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM9, &oc);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);

    Servo_SetPulse(SERVO_PE5_HOLD_US);
}

static void Servo_ApplyState(uint8_t state, uint16_t *pe5_us)
{
    if (state == SERVO_STATE_HOLD)
    {
        *pe5_us = SERVO_PE5_HOLD_US;
    }
    else if (state == SERVO_STATE_DELIVER)
    {
        *pe5_us = SERVO_PE5_DELIVER_US;
    }
    else
    {
        *pe5_us = SERVO_MID_US;
    }

    Servo_SetPulse(*pe5_us);
}

static const char *Servo_StateName(uint8_t state)
{
    if (state == SERVO_STATE_DELIVER) return "DELIVER";
    if (state == SERVO_STATE_HOLD)    return "HOLD";
    return "MID";
}

static void Servo_Show(uint8_t state, uint16_t pe5_us)
{
#if SERVO_DEBUG
    printf("ServoTest: state=%s PE5=%u us\r\n",
           Servo_StateName(state),
           (unsigned int)pe5_us);
#else
    (void)state;
    (void)pe5_us;
#endif
}

int main(void)
{
    uint8_t index = 0U;
    uint8_t state;
    uint16_t pe5_us;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    USART1_Init(19200);

    LED_RED   = SERVO_LED_OFF;
    LED_GREEN = SERVO_LED_OFF;

    Servo_Init();

#if SERVO_DEBUG
    printf("\r\n== ServoTest single: PE5/TIM9_CH1 ==\r\n");
#endif

#if SERVO_INSTALL_MODE
    Servo_SetPulse(SERVO_PE5_HOLD_US);
#if SERVO_DEBUG
    printf("ServoTest: INSTALL MODE PE5=%u HOLD\r\n",
           (unsigned int)SERVO_PE5_HOLD_US);
#endif
    LED_RED   = SERVO_LED_ON;
    LED_GREEN = SERVO_LED_OFF;

    for (;;)
    {
        delay_ms(1000);
    }
#else
    Servo_ApplyState(SERVO_STATE_HOLD, &pe5_us);
    Servo_Show(SERVO_STATE_HOLD, pe5_us);
    LED_RED = SERVO_LED_ON;
    delay_ms(SERVO_START_DELAY_MS);

    for (;;)
    {
        state = Servo_TestStates[index];
        Servo_ApplyState(state, &pe5_us);
        Servo_Show(state, pe5_us);

        if (state == SERVO_STATE_HOLD)
        {
            LED_RED   = SERVO_LED_ON;
            LED_GREEN = SERVO_LED_OFF;
        }
        else
        {
            LED_RED   = SERVO_LED_OFF;
            LED_GREEN = SERVO_LED_ON;
        }

        delay_ms(SERVO_HOLD_MS);
        index++;
        if (index >= SERVO_TEST_COUNT) index = 0U;
    }
#endif
}

/* 工程文件清单包含灰阶驱动，它们链接时需要 reverse()。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAAU) >> 1) | ((x & 0x55U) << 1));
    x = (uint8_t)(((x & 0xCCU) >> 2) | ((x & 0x33U) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}
