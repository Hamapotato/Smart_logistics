#include "pid.h"

PID_InitTypeDef PID_201, PID_202, PID_203, PID_204,SENSOR;
float Target_Angle;

void PID_Init(void)
{
    SENSOR.kp             = 0.1f;
    SENSOR.ki             = 0.0f;
    SENSOR.kd             = 0.01f;
    SENSOR.Pmax           = 15;
    SENSOR.Imax           = 15;
    SENSOR.Dmax           = 15;
    SENSOR.max_PIDoutput  = 15;
    SENSOR.last_error     = 0;
    SENSOR.pred_error     = 0;
    SENSOR.ignored_error  = 1;
    SENSOR.error_sum      = 0;
    SENSOR.last_PIDoutput = 0;
}

int16_t Positional_PID_calc(PID_InitTypeDef *pid, uint8_t Target_Speed,uint8_t Real_Speed)
{
    pid->error = Target_Speed - Real_Speed;
    if(fabs(pid->error) <= pid->ignored_error) pid->error = 0;
    pid->error_sum += pid->error;
    
    pid->Pout = pid->kp * pid->error;
    if(pid->Pout > pid->Pmax) pid->Pout = pid->Pmax;
    if(pid->Pout < -pid->Pmax) pid->Pout = -pid->Pmax;

    pid->Iout = pid->ki * pid->error_sum;
    if(pid->Iout > pid->Imax) pid->Iout = pid->Imax;
    if(pid->Iout < -pid->Imax) pid->Iout = -pid->Imax;

    pid->Dout = pid->kd * (pid->error - pid->last_error);
    if(pid->Dout > pid->Dmax) pid->Dout = pid->Dmax;
    if(pid->Dout < -pid->Dmax) pid->Dout = -pid->Dmax;

    pid->PIDoutput = pid->Pout + pid->Iout + pid->Dout;
    if(pid->PIDoutput > pid->max_PIDoutput) pid->PIDoutput = pid->max_PIDoutput;
    if(pid->PIDoutput < -pid->max_PIDoutput) pid->PIDoutput = -pid->max_PIDoutput;
    pid->last_error = pid->error;
    return pid->PIDoutput;
}

float Incremental_PID_calc(PID_InitTypeDef *pid, float Target_Speed,float Real_Speed)
{
    pid->error = Target_Speed - Real_Speed;
    if(fabsf(pid->error) <= pid->ignored_error) pid->error = 0;

    pid->Pout = pid->kp * (pid->error - pid->last_error);
    pid->Iout = pid->ki * pid->error;
    pid->Dout = pid->kd * (pid->error - 2 * pid->last_error + pid->pred_error);

    pid->PIDoutput += pid->Pout + pid->Iout + pid->Dout;
    if(pid->PIDoutput > pid->max_PIDoutput) pid->PIDoutput = pid->max_PIDoutput;
    if(pid->PIDoutput < -pid->max_PIDoutput) pid->PIDoutput = -pid->max_PIDoutput;

    pid->pred_error = pid->last_error;
    pid->last_error = pid->error;
    return pid->PIDoutput;
}