#ifndef _plan_H
#define _plan_H

#include "main.h"

#define TRANSIABILITY   0
#define UNTRANSIABILITY 1
#define INFINISS        99

typedef struct
{
    int8_t Start_To_End; // 到终点的代价
    int8_t End_To_Start; // 到起点的代价
    int8_t Cost;         // 总代价
    int8_t TRANSBILITY;  // 可达性
} Coordinate_InitTypeDef; // 坐标点结构体

typedef struct
{
    int8_t X;
    int8_t Y;
} Route_InitTypeDef; // 路径结构体

extern Coordinate_InitTypeDef Coordinate[8][5];
extern Route_InitTypeDef Route[40];

void Coordinate_Init(void);
void Coordinate_Refresh(void);
void Obstacle_Process(void);
u8 Route_Plan(void);

#endif