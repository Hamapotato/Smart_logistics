#ifndef _yysb_H
#define _yysb_H

#include "main.h"

#define SDA_PIN         GPIO_Pin_0
#define SCL_PIN         GPIO_Pin_9

#define SDA_CLOCK       RCC_AHB1Periph_GPIOE
#define SCL_CLOCK       RCC_AHB1Periph_GPIOB

#define SDA_PORT        GPIOE
#define SCL_PORT        GPIOB

#define Asr_Addr        0x34
#define Asr_Read        0x69

#define ASR_RESULT_ADDR 0x64
#define ASR_SPEAK_ADDR  0x6E

#define ASR_CMDMAND     0x00
#define ASR_ANNOUNCER   0xFF

void ASR_IIC_Init(void);
void ASR_IIC_Start(void);
void ASR_IIC_Stop(void);
void ASR_IIC_SendByte(uint8_t Byte);
uint8_t ASR_IIC_ReceiveByte(u8 Ack);
void ASR_IIC_SendAck(uint8_t AckBit);
uint8_t ASR_IIC_ReceiveAck(void);
void Asr_Speak(u8 cmd, u8 idNum);
u8 Asr_Result(void);

#endif // !yysb_H