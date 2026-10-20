#include "imu_driver.h"

void MyI2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOF, GPIO_Pin_3, (BitAction)BitValue);
    delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue) // 写SDA
{
    GPIO_WriteBit(GPIOF, GPIO_Pin_4, (BitAction)BitValue);
    delay_us(10);
}

uint8_t MyI2C_R_SDA(void) // 读SDA
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_4);
    // Delay_us(10);
    return BitValue;
}

void MyI2C_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; // 开漏输出
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_SetBits(GPIOF, GPIO_Pin_3 | GPIO_Pin_4);
}

// I2C启始条件
void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0); // SCL高电平时拉低SDA
    MyI2C_W_SCL(0);
}

// I2C结束条件
void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1); // SCL高电平时拉高SDA
    MyI2C_W_SDA(1);
}
// 主机发一个应答
void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

// 主机接受一个应答
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t i;
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    while (MyI2C_R_SDA()) {
        i++;
        if (i > 250) {
            MyI2C_Stop();
            return 1;
        }
    }

    MyI2C_W_SCL(0);
    return 0;
}

// I2C写一个字节（8位）
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        MyI2C_W_SDA(Byte & (0x80 >> i));
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
}

// 读IIC读一个字节
uint8_t MyI2C_ReceiveByte(u8 Ack)
{
    uint8_t i, Byte = 0x00;
    MyI2C_W_SDA(1); // 主机拉高SDA方便从机写入数据
    for (i = 0; i < 8; i++) {
        MyI2C_W_SCL(1);
        if (MyI2C_R_SDA() == 1) { Byte |= (0x80 >> i); }
        MyI2C_W_SCL(0);
    }
    MyI2C_SendAck(Ack);
    return Byte;
}