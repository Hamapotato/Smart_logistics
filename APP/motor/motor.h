#ifndef _motor_H
#define _motor_H

#include "main.h"

#define IIC_SCL                     PFout(6) // SCL
#define IIC_SDA                     PFout(5) // 输出SDA
#define READ_SDA                    PFin(5)  // 输入SDA

#define CAM_DEFAULT_I2C_ADDRESS     0x34 // I2C地址
#define MOTOR_TYPE_ADDR             0x14 // 编码电机类型设置寄存器地址
#define MOTOR_FIXED_SPEED_ADDR      0x33 // 速度寄存器地址。属于闭环控制
#define MOTOR_ENCODER_POLARITY_ADDR 0x15 // 电机编码方向极性地址
#define MOTOR_FIXED_PWM_ADDR        0x1F // 固定PWM控制地址，属于开环控制
#define MOTOR_ENCODER_TOTAL_ADDR    0x3C // 4个编码电机各自的总脉冲值

/* --------------------------------------------------------------- */
/* Hiwonder 4-channel encoder motor driver.
   Channels 1..4 drive the four JGB37-520 chassis motors.
   This board has no fifth motor channel. */
#define MOTOR_COUNT                 4

/* Register address of ONE channel's encoder total count, so a single
   channel costs 4 bytes instead of 4*ch.  If the board does not
   decode per-channel offsets, define MOTOR_ENCODER_BLOCK_READ in
   motor.c's MOTOR_Encoder_Read() instead.                        */
#define MOTOR_ENCODER_CH_ADDR(ch)   (MOTOR_ENCODER_TOTAL_ADDR + 4 * ((ch) - 1))

/* Returned by MOTOR_Encoder_Read() when the I2C transfer failed. */
#define MOTOR_ENCODER_ERR           ((int32_t)0x80000000)
#define ADC_BAT_ADDR                0x00 // 电压地址

// 电机类型具体地址
#define MOTOR_TYPE_WITHOUT_ENCODER 0 // 无编码器的电机,磁环每转是44个脉冲减速比:90  默认
#define MOTOR_TYPE_TT              1 // TT编码电机
#define MOTOR_TYPE_N20             2 // N20编码电机
#define MOTOR_TYPE_JGB             3 // JGB37-520（Hiwonder 官方类型 3）

typedef enum {
    STOP,
    FRONT,
    BACK,
    RIGHT,
    LEFT,
    REVOLVE
} DIRECTION_MODE;

typedef struct
{
    float Target_Speed;
    float Real_Speed;
    float PID_Output;
} Motor_InitTypeDef;

extern Motor_InitTypeDef Motor_201, Motor_202, Motor_203, Motor_204; // JGB37-520 四轮电机
extern DIRECTION_MODE Direction;

void IIC_init(void);
uint8_t I2C_Read_Len(uint8_t Reg, uint8_t *Buf, uint8_t Len);
uint8_t I2C_Write_Len(uint8_t Reg, int8_t *Buf, uint8_t Len);
void MOTOR_Init(void);
void MOTOR_CONTROL_DATA_Predit(Motor_InitTypeDef *M1, Motor_InitTypeDef *M2, Motor_InitTypeDef *M3, Motor_InitTypeDef *M4);
int32_t MOTOR_Encoder_Read(u8 ch);

void MOTOR_ENCODER_DATA_Predit(void);

#endif