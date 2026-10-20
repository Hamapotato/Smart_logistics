#include "yysb.h"

void ASR_IIC_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(SCL_PORT, SCL_PIN, (BitAction)BitValue);
    delay_us(10);
}

void ASR_IIC_W_SDA(uint8_t BitValue) // 写SDA
{
    GPIO_WriteBit(SDA_PORT, SDA_PIN, (BitAction)BitValue);
    delay_us(10);
}

uint8_t ASR_IIC_R_SDA(void) // 读SDA
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN);
    // Delay_us(10);
    return BitValue;
}

void ASR_IIC_Init(void)
{
    RCC_AHB1PeriphClockCmd(SDA_CLOCK, ENABLE);
    RCC_AHB1PeriphClockCmd(SCL_CLOCK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; // 开漏输出
    GPIO_InitStructure.GPIO_Pin   = SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(SDA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; // 开漏输出
    GPIO_InitStructure.GPIO_Pin   = SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);

    GPIO_SetBits(SDA_PORT, SDA_PIN);
    GPIO_SetBits(SCL_PORT, SCL_PIN);
}

// I2C启始条件
void ASR_IIC_Start(void)
{
    ASR_IIC_W_SDA(1);
    ASR_IIC_W_SCL(1);
    ASR_IIC_W_SDA(0); // SCL高电平时拉低SDA
    ASR_IIC_W_SCL(0);
}

// I2C结束条件
void ASR_IIC_Stop(void)
{
    ASR_IIC_W_SDA(0);
    ASR_IIC_W_SCL(1); // SCL高电平时拉高SDA
    ASR_IIC_W_SDA(1);
}
// 主机发一个应答
void ASR_IIC_SendAck(uint8_t AckBit)
{
    ASR_IIC_W_SDA(AckBit);
    ASR_IIC_W_SCL(1);
    ASR_IIC_W_SCL(0);
}

// 主机接受一个应答
uint8_t ASR_IIC_ReceiveAck(void)
{
    uint8_t i;
    ASR_IIC_W_SDA(1);
    ASR_IIC_W_SCL(1);
    while (ASR_IIC_R_SDA()) {
        i++;
        if (i > 250) {
            ASR_IIC_Stop();
            return 1;
        }
    }

    ASR_IIC_W_SCL(0);
    return 0;
}

// I2C写一个字节（8位）
void ASR_IIC_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        ASR_IIC_W_SDA(Byte & (0x80 >> i));
        ASR_IIC_W_SCL(1);
        ASR_IIC_W_SCL(0);
    }
}

// 读IIC读一个字节
uint8_t ASR_IIC_ReceiveByte(u8 Ack)
{
    uint8_t i, Byte = 0x00;
    ASR_IIC_W_SDA(1); // 主机拉高SDA方便从机写入数据
    for (i = 0; i < 8; i++) {
        ASR_IIC_W_SCL(0);
        ASR_IIC_W_SCL(1);
        Byte <<= 1;
        if (ASR_IIC_R_SDA() == 1) { Byte++; }
    }
    ASR_IIC_SendAck(Ack);
    return Byte;
}

// 设置播报
void Asr_Speak(u8 cmd, u8 idNum)
{
    int i;
    u8 send[2] = {0x00, 0x00};
    if (cmd == 0xFF || cmd == 0x00) {
        send[0] = cmd;
        send[1] = idNum;
        ASR_IIC_Start();                     // 起始信号
        ASR_IIC_SendByte(Asr_Addr << 1 | 0); // 发送写指令
        ASR_IIC_ReceiveAck();                // 应答
        ASR_IIC_SendByte(ASR_SPEAK_ADDR);    // 发送添加词条指令
        ASR_IIC_ReceiveAck();                // 应答
        // IIC_Stop();
        for (i = 0; i < 2; ++i) {
            ASR_IIC_SendByte(send[i]);
            ASR_IIC_ReceiveAck(); // 应答
        }
        ASR_IIC_Stop(); // 停止
        delay_ms(20);
    }
}

// 获取识别结果
u8 Asr_Result(void)
{
    u8 result = 0;
    ASR_IIC_Start();                     // 起始信号
    ASR_IIC_SendByte(Asr_Addr << 1 | 0); // 发送写指令
    ASR_IIC_ReceiveAck();                // 应答
    ASR_IIC_SendByte(ASR_RESULT_ADDR);   // 识别结果存放处，通过不断读取此地址的值判断是否识别到语音，不同的值对应不同的语音，
    ASR_IIC_ReceiveAck();                // 应答
    // ASR_IIC_Stop(); //停止
    ASR_IIC_Start();                     // 起始信号
    ASR_IIC_SendByte(Asr_Addr << 1 | 1); // 发送读指令
    ASR_IIC_ReceiveAck();                // 应答
    result = ASR_IIC_ReceiveByte(0);     // 读取一个字节
    ASR_IIC_Stop();                      // 停止
    delay_ms(20);
    return result;
}