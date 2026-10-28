/* ============================================================================
 * ServoTest —— 单路舵机 PWM 测试
 *
 * 信号：PE5 / TIM9_CH1
 * PWM：50 Hz，20 ms 周期
 * 默认脉冲：800 us、1500 us、1700 us
 *
 * 测试顺序：
 *   800 us -> 1500 us -> 1700 us -> 1500 us -> 循环
 * ========================================================================== */

#include "main.h"

/* 舵机 PWM 脉宽，单位为 TIM 计数。定时器配置成 1 MHz，因此 1 计数 = 1 us。 */
#define SERVO_MIN_US                 800U
#define SERVO_MID_US                 1500U
#define SERVO_MAX_US                 1700U

/* 上电后等待时间，给机构安装和人员撤离留时间。 */
#define SERVO_START_DELAY_MS         2000U

/* 每个测试位置的保持时间。 */
#define SERVO_HOLD_MS                1500U

/* 1 = 输出串口日志。 */
#define SERVO_DEBUG                  1

#define SERVO_LED_ON                 0
#define SERVO_LED_OFF                1

#if (SERVO_MIN_US >= SERVO_MID_US)
  #error "SERVO_MIN_US must be smaller than SERVO_MID_US"
#endif

#if (SERVO_MID_US >= SERVO_MAX_US)
  #error "SERVO_MID_US must be smaller than SERVO_MAX_US"
#endif

#define SERVO_TEST_COUNT             4U

static const uint16_t Servo_TestPulses[SERVO_TEST_COUNT] =
{
    SERVO_MIN_US,
    SERVO_MID_US,
    SERVO_MAX_US,
    SERVO_MID_US
};

static void Servo_SetPulse(uint16_t pulse)
{
    if (pulse < SERVO_MIN_US) pulse = SERVO_MIN_US;
    if (pulse > SERVO_MAX_US) pulse = SERVO_MAX_US;
    TIM_SetCompare1(TIM9, pulse);
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
    oc.TIM_Pulse       = SERVO_MIN_US;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM9, &oc);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);

    Servo_SetPulse(SERVO_MIN_US);
}

static void Servo_Show(uint16_t pulse)
{
#if SERVO_DEBUG
    printf("ServoTest: pulse=%u us\r\n", (unsigned int)pulse);
#else
    (void)pulse;
#endif
}

int main(void)
{
    uint8_t index = 0;
    uint16_t pulse;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    USART1_Init(19200);

    LED_RED   = SERVO_LED_OFF;
    LED_GREEN = SERVO_LED_OFF;

    Servo_Init();
    Servo_SetPulse(SERVO_MIN_US);

#if SERVO_DEBUG
    printf("\r\n== ServoTest: PE5 / TIM9_CH1, 50 Hz ==\r\n");
    printf("ServoTest: range=%u..%u us, start in %u ms\r\n",
           (unsigned int)SERVO_MIN_US, (unsigned int)SERVO_MAX_US,
           (unsigned int)SERVO_START_DELAY_MS);
#endif

    LED_GREEN = SERVO_LED_ON;
    delay_ms(SERVO_START_DELAY_MS);

    for (;;)
    {
        pulse = Servo_TestPulses[index];
        Servo_SetPulse(pulse);
        Servo_Show(pulse);

        if (pulse == SERVO_MAX_US)
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
        if (index >= SERVO_TEST_COUNT) index = 0;
    }
}

/* 工程文件清单包含灰阶驱动，它们链接时需要 reverse()。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAAU) >> 1) | ((x & 0x55U) << 1));
    x = (uint8_t)(((x & 0xCCU) >> 2) | ((x & 0x33U) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}
