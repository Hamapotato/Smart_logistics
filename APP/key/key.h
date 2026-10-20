#ifndef _key_H
#define _key_H

#include "main.h"

#define KEY_ROUTE PEin(9)
#define KEY_TEAM PDin(10)

void ROUTE_CHOSE_KEY_Init(void);
void TEAM_CHOSE_KEY_Init(void);

extern u8 ROUTE_CHOSE; // 0 优先横向运动 1 优先纵向运动
extern u8 TEAM_CHOSE; // 0 蓝方 1 红方

#endif