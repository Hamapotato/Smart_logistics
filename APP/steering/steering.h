#ifndef _steering_H
#define _steering_H

#include "main.h"

/* -------------------------------------------------------------------------
 * 抛投机构当前使用原单路舵机接口：
 *     PE5 / TIM9_CH1，50 Hz PWM
 *
 * Hiwonder 0x34 四路编码电机驱动板只控制四个 JGB37-520 底盘电机，
 * 没有第 5 路，因此不再使用 Motor_205、0x33 第 5 字节或 0x4C。
 * ------------------------------------------------------------------------- */

#define ARM_HOME_ON_START   0

void Steering_Init(void);
void Steering_RESET(void);
void Steering_SET(void);
void Steering_Home(void);
void Steering_Task(void);
void Engine_Working(void);

extern volatile u8 STEERING_FLAG;

#endif
