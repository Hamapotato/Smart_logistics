#include "steering.h"

/* 抛投机构使用原 PE5 / TIM9_CH1 单路舵机接口。
   Hiwonder 0x34 四路编码电机驱动板只驱动四个 JGB37-520 底盘电机，
   没有第 5 路。 */


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
