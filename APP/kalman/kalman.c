#include "kalman.h"

KALMAN_InitTypeDef kalman_yaw;

void Kalman_Init(KALMAN_InitTypeDef* this)
{
    this->X = MPU9250_DATA.yaw;
    this->Z = 0;
    this->T = 0.01;
    this->Q_angle = 0.001;
    this->Q_gyro = 0.003;
    this->R_angle = 0.5;
    this->Q_bias = 0;
    this->P[0][0] = 1;
    this->P[0][1] = 0;
    this->P[1][0] = 0;
    this->P[1][1] = 1;
    this->K_0 = 0;
    this->K_1 = 0;
}

float Kalman_Update(KALMAN_InitTypeDef* this,float gyro,float yaw_g)
{
    this->X += this->T * (gyro - this->Q_bias);
    this->Z = yaw_g;
    this->P[0][0] = this->P[0][0] + this->Q_angle - (this->P[0][1] + this->P[1][0]) * this->T;
    this->P[0][1] = this->P[0][1] - this->P[1][1] * this->T;
    this->P[1][0] = this->P[1][0] - this->P[1][1] * this->T;
    this->P[1][1] = this->P[1][1] + this->Q_gyro;
    this->K_0 = this->P[0][0] / (this->P[0][0] + this->R_angle);
    this->K_1 = this->P[1][0] / (this->P[0][0] + this->R_angle);
    this->X = this->X + this->K_0 * (this->Z - this->X);
    this->Q_bias = this->Q_bias + this->K_1 * (this->Z - this->X);
    this->P[0][0] = this->P[0][0] - this->K_0 * this->P[0][0];
    this->P[0][1] = this->P[0][1] - this->K_0 * this->P[0][1];
    this->P[1][0] = this->P[1][0] - this->K_1 * this->P[0][0];
    this->P[1][1] = this->P[1][1] - this->K_1 * this->P[0][1];
    return this->X;
}