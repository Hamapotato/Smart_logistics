#ifndef _tubes_H
#define _tubes_H

#include "main.h"

#define TUBES_W_CLK PCout(5)
#define TUBES_W_DIO PCout(4)
#define TUBES_R_DIO PCin(4)

void TUBES_Driver_Init(void);
void TUBES_Display(void);
void TUBES_SetBrightness(unsigned char level);
void TUBES_SetValue(u16 value); // 设置数码管显示的数字（0-9999）

extern unsigned char disp_num[4];
extern unsigned char tab[18];

#endif