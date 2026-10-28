#ifndef _trace_H
#define _trace_H

#define CHASSIS_OFF_VALUE_1      2
#define CHASSIS_OFF_VALUE_2      4
#define CHASSIS_OFF_VALUE_3      6
#define CHASSIS_OFF_VALUE_4      8
#define CHASSIS_OMEGA_VALUE_LOW  2
#define CHASSIS_OMEGA_VALUE_MID  4
#define CHASSIS_OMEGA_VALUE_HIGH 6
#define CHASSIS_RATIO

/* ---------------------------------------------------------------------------
   巡线纠偏量作用在哪个轮速组合上
   ---------------------------------------------------------------------------
   本车底盘是对角式全向轮，轮速空间是一组 Hadamard 基，四个组合两两正交，
   只有三组是真运动（2026-10-10 用 MoveTest 实测确认）：

      前进/后退  (+1,+1,+1,+1)   平移  <- FRONT_BACK_TRACING 的基础量
      左移/右移  (+1,-1,+1,-1)   平移  <- RIGHT_LEFT_TRACING 的基础量
      原地旋转   (+1,+1,-1,-1)   旋转  <- Off_Value / Revolve 用的就是这一组
      空转       (+1,-1,-1,+1)   不动  <- 对角轮对顶，合力合力矩都是 0

   实测：201=右前 202=右后 203=左后 204=左前；四个电机的“正转”都是车轮前进方向；
   驱动板认正负号（正转反转真的相反）。Off_Value 按 (-1,-1,+1,+1)（“原地旋转”
   的反相）加到四个轮子上，纠偏就是把车头扭回黑线方向。（曾经有一版误把空转组
   (+1,-1,-1,+1) 当成旋转改了进去，那样纠偏量完全不起作用；Revolve() 原来也用的
   空转组，同样发下去不动。）

   TRACE_STEER_SIGN：纠偏方向反了（车头往远离黑线的方向扭）就改成 -1。
--------------------------------------------------------------------------- */
#define TRACE_STEER_SIGN  (+1)

/* 纠偏量（已带符号）：正向 = 按“原地旋转”组合把车头扭向黑线。
   speed 为 0（停车/待命）时不纠偏，免得车在原地自己转起来。 */
#define TRACE_OFF(spd)    ((spd) ? (TRACE_STEER_SIGN * Off_Value) : 0)

#include "main.h"

void FRONT_BACK_TRACING(int8_t speed, u8 mode);
void RIGHT_LEFT_TRACING(int8_t speed, u8 mode);
uint8_t reverse(uint8_t x);
void Revolve(u8 speed);

#endif