#ifndef _sensor_h
#define _sensor_h

#include "main.h"

#define FRONT_GREY_SENSOR_READ  F_GW_digital_Read()
#define BACK_GREY_SENSOR_READ   B_GW_digital_Read()
#define LEFT_GREY_SENSOR_READ   L_GW_digital_Read()
#define RIGHT_GREY_SENSOR_READ  R_GW_digital_Read()

#define FRONT_L_OBSTACLE        !GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)
#define FRONT_R_OBSTACLE        !GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1)
#define BACK_L_OBSTACLE         !GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_4)
#define BACK_R_OBSTACLE         !GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)
#define LEFT_U_OBSTACLE         !GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)
#define LEFT_D_OBSTACLE         !GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5)
#define RIGHT_U_OBSTACLE        !GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_3)
#define RIGHT_D_OBSTACLE        !GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_2)

#define START_FLAG GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12)

typedef struct
{
    uint8_t FRONT_SENSOR_COUNT;
    uint8_t BACK_SENSOR_COUNT;
    uint8_t LEFT_SENSOR_COUNT;
    uint8_t RIGHT_SENSOR_COUNT;

    uint8_t FRONT_SENSOR;
    uint8_t BACK_SENSOR;
    uint8_t LEFT_SENSOR;
    uint8_t RIGHT_SENSOR;

    uint8_t FRONT_SENSOR_INFORMATION;
    uint8_t BACK_SENSOR_INFORMATION;
    uint8_t LEFT_SENSOR_INFORMATION;
    uint8_t RIGHT_SENSOR_INFORMATION;
} GREY_InitTypeDef;

typedef struct
{
    u8 FRONT_OBSTACLE_COUNT;
    u8 BACK_OBSTACLE_COUNT;
    u8 LEFT_OBSTACLE_COUNT;
    u8 RIGHT_OBSTACLE_COUNT;
    u8 OBSTACLE_INFORMATION;
} OBSTACLE_InitTypeDef;

extern GREY_InitTypeDef GREY; // »Ò¶È¶¨Î»
extern OBSTACLE_InitTypeDef OBSTACLE;
extern u8 BLUE_FLAG;
extern u8 RED_FLAG;

void GREY_Init(void);
void OBSTACLE_Init(void);
void INFRAED_Init(void);

#endif