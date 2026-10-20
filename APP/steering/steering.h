#ifndef _steering_H
#define _steering_H

#include "main.h"

/* =========================================================================
 * 投递机构。二选一：
 *   ARM_USE_I2C_MOTOR = 1  JGB520 编码器减速电机，接在 0x34 电机驱动板的
 *                          ARM_CH 路上（1~4 是四轮）。编码器做位置闭环，
 *                          行程终点靠堵转判定。
 *   ARM_USE_I2C_MOTOR = 0  原来的 MG996R 舵机（TIM9_CH1 / PE5，50Hz PWM）。
 * 翻这个宏即可切换，main.c / move.c 都不用改。
 * ========================================================================= */
#define ARM_USE_I2C_MOTOR   1

#if ARM_USE_I2C_MOTOR

/* ---- 机械 / 电气 ------------------------------------------------------ */
#define ARM_CH              5       /* 驱动板通道号（1 起算），1~4 是四轮    */
#define ARM_DIR_SIGN        1       /* 若正指令让编码器计数变小，改成 -1    */

/* ---- 行程与增益（上车前必须标定） ------------------------------------- */
/* 复位位到抛出位之间的编码器计数差，带符号：正 = 抛出方向让计数增大。
   参考：JGB 系列按「每转 44 线 x 减速比 90」约 3960 counts/转，120 度行程
   约 1320。先给个小值试，用量角器量实际转角再换算。
   若抛出方向有硬限位，这个值只是兜底，真正停靠靠堵转。 */
#define ARM_THROW_COUNTS    1320
#define ARM_KP              0.05f   /* 位置环：cmd = KP*err - KD*vel         */
#define ARM_KD              0.30f   /* 微分作用在测得的转速上，不是误差上    */
#define ARM_MAX_CMD         30      /* 指令幅值上限（与四轮同一单位）        */
#define ARM_ARRIVE_TOL      8       /* 计数误差落进这个范围就算到位          */

/* ---- 堵转判定（行程终点） --------------------------------------------- */
#define ARM_STALL_EPS       2       /* 一个 30ms 周期里计数变化 <= 此值算没动 */
#define ARM_STALL_CMD       4       /* ...且当时指令绝对值 >= 此值才算堵转    */
#define ARM_STALL_N         3       /* 连续几个周期成立就认定到行程终点       */
#define ARM_HOME_CMD        5       /* 回零爬行速度（低速，压住堵转电流）     */

/* ---- 超时与容错 ------------------------------------------------------- */
#define ARM_HOME_TIMEOUT_MS 4000    /* 回零超时：爬行距离长，单独放宽   */
#define ARM_TIMEOUT_MS      1500    /* 单次动作超过这么多毫秒就报故障         */
#define ARM_IO_FAIL_MAX     10      /* 连续读编码器失败这么多次就报故障       */

/* 发车时是否重新回零一次。1 = 回（更保险，多花约 0.1~1s）；
   0 = 只在开机时回零一次。 */
#define ARM_HOME_ON_START   1

#else

#define ARM_HOME_ON_START   0

#endif

void Steering_Init(void);
void Steering_RESET(void);
void Steering_SET(void);
void Steering_Home(void);
void Steering_Task(void);
void Engine_Working(void);

extern volatile u8 STEERING_FLAG;

#endif
