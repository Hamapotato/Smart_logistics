#include "motor.h"

Motor_InitTypeDef Motor_201, Motor_202, Motor_203, Motor_204;
int8_t MOTOR_Data[MOTOR_COUNT];
int8_t MotorEncoderPolarity; // 电机极性控制变量
uint32_t EncodeTotal[4];      // 用于暂存电机累积转动量的值，正转递增，反转递减
int8_t MotorType;            // 设置电机类型
DIRECTION_MODE Direction;

void I2C_SDA_OUT(void) // SDA输出方向配置
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; // SCL上拉输入
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推??输出
    GPIO_Init(GPIOF, &GPIO_InitStructure);
}

void I2C_SDA_IN(void) // SDA输入方向配置
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; // SCL上拉输入
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; // 上拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);
}

// 以下为模拟IIC总线函数
void IIC_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); // 使能PD端口时钟

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6; // PD6配置为推挽输出,SCL
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    // 推??输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // IO口速度为50MHz
    GPIO_Init(GPIOF, &GPIO_InitStructure);            // 根据设定参数初始化GPIOD
    GPIO_SetBits(GPIOF, GPIO_Pin_5 | GPIO_Pin_6);
}

void IIC_start() // 起始信号
{
    I2C_SDA_OUT();
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(5);
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 0;
}

void IIC_stop() // 终止信号
{
    I2C_SDA_OUT();
    IIC_SCL = 0;
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 1;
    IIC_SDA = 1;
    delay_us(5);
}

// 主机产生一个应答信号
void IIC_ack()
{
    IIC_SCL = 0;
    I2C_SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

// 主机不产生应答信号
void IIC_noack()
{
    IIC_SCL = 0;
    I2C_SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

// 等待从机应答信号
// 返回值：1 接收应答失败
//		  0 接收应答成功
u8 IIC_wait_ack()
{
    u8 tempTime = 0;
    I2C_SDA_IN();
    IIC_SDA = 1;
    delay_us(1);
    IIC_SCL = 1;
    delay_us(1);

    while (READ_SDA) {
        tempTime++;
        if (tempTime > 250) {
            IIC_stop();
            return 1;
        }
    }

    IIC_SCL = 0;
    return 0;
}

void IIC_send_byte(u8 txd)
{
    u8 i = 0;
    I2C_SDA_OUT();
    IIC_SCL = 0;
    ; // 拉低时钟开始数据传输
    for (i = 0; i < 8; i++) {
        IIC_SDA = (txd & 0x80) >> 7; // 读取字节
        txd <<= 1;
        delay_us(2);
        IIC_SCL = 1;
        delay_us(2); // 发送数据
        IIC_SCL = 0;
        delay_us(2);
    }
}

// 读取一个字节
u8 IIC_read_byte(u8 ack)
{
    u8 i = 0, receive = 0;
    I2C_SDA_IN();
    for (i = 0; i < 8; i++) {
        IIC_SCL = 0;
        delay_us(2);
        IIC_SCL = 1;
        receive <<= 1; // 左移
        if (READ_SDA)
            receive++; // 连续读取八位
        delay_us(1);
    }

    if (!ack)
        IIC_noack();
    else
        IIC_ack();

    return receive; // 返回读取到的字节
}

// 读取多个字节的数据（Reg：地址  Buf：数据内容 Len：数据长度）
// 读取多个字节的数据（Reg：地址  Buf：数据内容 Len：数据长度）
uint8_t I2C_Read_Len(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    uint8_t i;
    IIC_start();                                       // 发送起始信号
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0); // 发送元器件地址+写指令
    if (IIC_wait_ack() == 1)                           // 等待响应，如果失败，发送停止信号，返回1
    {
        //printf("hujiao_fall:%d\r\n",i);
        IIC_stop();
        return 1;
    }
    IIC_send_byte(Reg);      // 发送寄存器地址
    if (IIC_wait_ack() == 1) // 等待响应，如果失败，发送停止信号，返回1
    {
        //printf("encoder_fall:%d\r\n",i);
        IIC_stop();
        return 1;
    }
    IIC_start();                                       // 发送起始信号
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 1); // 发送元器件地址+读指令
    if (IIC_wait_ack() == 1)                           // 等待响应，如果失败，发送停止信号，返回1
    {
        //printf("hujiao_fall:%d\r\n",i);
        IIC_stop();
        return 1;
    }
    for (i = 0; i < Len; i++) // for循环Len次读取
    {
        if (i != Len - 1) // 如果不是最后一次
        {
            Buf[i] = IIC_read_byte(1); // 保存第I次的数据到数组的第I位，并发送答应信号
        } else
            Buf[i] = IIC_read_byte(0); // 保存第I次的数据到数组的第I位，并发送非答应信号
    }
    IIC_stop(); // 发送停止信号
    return 0;   // 读取成功，返回0
}

// 循环发送一个数组的数据（addr：地址  buf：数据内容  leng：数据长度）
uint8_t I2C_Write_Len(uint8_t Reg, int8_t *Buf, uint8_t Len) // I2C的写数据
{
    uint8_t i;
    IIC_start();                                       // 在起始信号后必须发送一个7位从机地址+1位方向位，用“0”表示主机发送数据，“1”表示主机接收数据。
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0); // 发送 器件地址+写的命令
    if (IIC_wait_ack() == 1)                           // 等待响应，如果失败，发送停止信号，返回1
    {
        //printf("hujiao_fall:%d\r\n",i);
        IIC_stop();
        return 1;
    }
    IIC_send_byte(Reg);      // 发送 寄存器地址
    if (IIC_wait_ack() == 1) // 等待响应，如果失败，发送停止信号，返回1
    {
        IIC_stop();
        return 1;
    }
    for (i = 0; i < Len; i++) // 循环 len 次写数据
    {
        IIC_send_byte(Buf[i]);   // 发送第i位的8位数据
        if (IIC_wait_ack() == 1) // 等待响应，如果失败，发送停止信号，返回1
        {
            IIC_stop();
            return 1;
        }
    }
    IIC_stop(); // 发送结束，发送停止信号
    return 0;   // 返回 0，确定发送成功
}

/* Read one channel's accumulated encoder count (1-based channel number).
   Returns MOTOR_ENCODER_ERR if the transfer failed.

   The board reports little-endian int32 per channel at 0x3C, so the
   default path just reads 4 bytes at 0x3C + 4*(ch-1).  If this board
   only decodes the start of the block, enable the fallback below. */
int32_t MOTOR_Encoder_Read(u8 ch)
{
    uint8_t buf[4 * MOTOR_COUNT];
    uint32_t v;
    uint8_t *p;

    if ((ch < 1) || (ch > MOTOR_COUNT)) return MOTOR_ENCODER_ERR;

#ifdef MOTOR_ENCODER_BLOCK_READ
    if (I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR, buf, 4 * ch) != 0) return MOTOR_ENCODER_ERR;
    p = &buf[4 * (ch - 1)];
#else
    if (I2C_Read_Len(MOTOR_ENCODER_CH_ADDR(ch), buf, 4) != 0) return MOTOR_ENCODER_ERR;
    p = buf;
#endif

    v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

void MOTOR_Init(void)
{
    IIC_init();
    MotorEncoderPolarity = 1;                                               // 电机极性控制变量
    MotorType            = MOTOR_TYPE_JGB;                                      // 设置电机类型
    I2C_Write_Len(MOTOR_TYPE_ADDR, &MotorType, 1); // 设置电机类型
    I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1); // 设置电机极性设置
    delay_ms(5);
}

void MOTOR_CONTROL_DATA_Predit(Motor_InitTypeDef *M1, Motor_InitTypeDef *M2, Motor_InitTypeDef *M3, Motor_InitTypeDef *M4)
{
    MOTOR_Data[0] = M1->PID_Output;
    MOTOR_Data[1] = M2->PID_Output;
    MOTOR_Data[2] = M3->PID_Output;
    MOTOR_Data[3] = M4->PID_Output;
    I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, MOTOR_COUNT); // 控制电机转动
}

void MOTOR_ENCODER_DATA_Predit(void)
{
    I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);
    float Data[4];
    Data[0] = (float)((int16_t)EncodeTotal[0]);
    Data[1] = (float)((int16_t)EncodeTotal[1]);
    Data[2] = (float)((int16_t)EncodeTotal[2]);
    Data[3] = (float)((int16_t)EncodeTotal[3]);
    JustFloat_Send(Data, 4, USART1);
    
}
