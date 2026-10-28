#ifndef _move_H
#define _move_H

#include "main.h"

typedef struct
{
    int8_t X_Start; //起始点X坐标
    int8_t Y_Start; //起始点Y坐标
    int8_t X_Aim;   //目标点X坐标
    int8_t Y_Aim;   //目标点Y坐标
    int8_t X_Now;   //当前点X坐标
    int8_t Y_Now;   //当前点Y坐标
    int8_t X_End;   //终点X坐标
    int8_t Y_End;   //终点Y坐标
} Location_InitTypeDef;  //位置结构体

void Task_Start(void);
void TIM7_Init(void);

void go(void);
void ba(void);


extern u8 k;
extern Location_InitTypeDef Location;
extern u8 ROUTE_PLAN_FLAG;
extern u8 FLAG,Occupy_Flag;
extern volatile u32 TIM7_Tick_MS;   /* free-running ms counter, +5 per TIM7 IRQ */
extern volatile u8  TIM7_TICK_FLAG; /* set every 30ms, cleared by the consumer  */
extern u8           TASK_RUN_FLAG;  /* 1 once Task_Start() ran: enable the planner */



#endif