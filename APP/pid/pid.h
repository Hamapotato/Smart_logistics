#ifndef _pid_H
#define _pid_H

#include "main.h"

extern float Target_Angle;

typedef struct
{
    float kp;
    float ki;
    float kd;

    int16_t Pmax;
    int16_t Imax;
    int16_t Dmax;

    int16_t Pout;
    int16_t Iout;
    int16_t Dout;

    int16_t error;
    int16_t error_sum;
    int16_t last_error;
    int16_t pred_error;
    int16_t ignored_error;

    u32 output;
    int16_t max_PIDoutput;
    int16_t PIDoutput;
    int16_t last_PIDoutput;
}PID_InitTypeDef;

extern PID_InitTypeDef PID_201, PID_202, PID_203, PID_204,SENSOR;//PID½á¹¹Ìå

void PID_Init(void);
int16_t Positional_PID_calc(PID_InitTypeDef *pid, uint8_t Target_Speed,uint8_t Real_Speed);
float Incremental_PID_calc(PID_InitTypeDef *pid, float Target_Speed,float Real_Speed);

#endif