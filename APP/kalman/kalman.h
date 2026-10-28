#ifndef _kalman_H
#define _kalman_H

#include "main.h"

#define PI 3.141592653589793f

typedef struct
{
    float X;
    float Q_angle;
    float Q_gyro;
    float R_angle;
    float Q_bias;
    float K_0,K_1,Z;
    float P[2][2];
    float T;
}KALMAN_InitTypeDef;

void Kalman_Init(KALMAN_InitTypeDef* this);
float Kalman_Update(KALMAN_InitTypeDef* this,float gyro,float yaw_g);

extern KALMAN_InitTypeDef kalman_yaw;

#endif