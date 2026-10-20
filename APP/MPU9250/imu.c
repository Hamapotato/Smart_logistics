#include "imu.h"

MPU9250_DATA_InitTypeDef MPU9250_DATA;

uint8_t MPU9250_WriteReg(uint8_t ADDR, uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(ADDR << 1 | 0);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    if (MyI2C_ReceiveAck()) {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(Data);
    if (MyI2C_ReceiveAck()) {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_Stop();
    return 0;
}
/*************************
指定地址读
***************************/
uint8_t MPU9250_ReadReg(uint8_t ADDR, uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(ADDR << 1 | 0);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(ADDR << 1 | 1);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte(1);
    // MyI2C_SendAck(1);
    MyI2C_Stop();

    return Data;
}

u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    MyI2C_Start();
    MyI2C_SendByte((addr << 1) | 0); // 发送器件地址+写命令
    if (MyI2C_ReceiveAck())          // 等待应答
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(reg); // 写寄存器地址
    MyI2C_ReceiveAck();  // 等待应答
    for (i = 0; i < len; i++) {
        MyI2C_SendByte(buf[i]); // 发送数据
        if (MyI2C_ReceiveAck()) // 等待ACK
        {
            MyI2C_Stop();
            return 1;
        }
    }
    MyI2C_Stop();
    return 0;
}

// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
// 返回值:0,正常
//     其他,错误代码
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    MyI2C_Start();
    MyI2C_SendByte((addr << 1) | 0); // 发送器件地址+写命令
    if (MyI2C_ReceiveAck())          // 等待应答
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(reg); // 写寄存器地址
    MyI2C_ReceiveAck();  // 等待应答
    MyI2C_Start();
    MyI2C_SendByte((addr << 1) | 1); // 发送器件地址+读命令
    MyI2C_ReceiveAck();              // 等待应答
    while (len) {
        if (len == 1)
            *buf = MyI2C_ReceiveByte(0); // 读数据,发送nACK
        else
            *buf = MyI2C_ReceiveByte(1); // 读数据,发送ACK
        len--;
        buf++;
    }
    MyI2C_Stop(); // 产生一个停止条件
    return 0;
}

u8 MPU9250_Init(void)
{
    u8 res = 0;
    MyI2C_Init();
    MPU9250_WriteReg(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X80); // 复位MPU9250
    delay_ms(100);                                         // 延时100ms
    MPU9250_WriteReg(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU9250
    MPU_Set_Gyro_Fsr(3);                                   // 陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                                  // 加速度传感器,±2g
    MPU_Set_Rate(50);                                      // 设置采样率50Hz
    MPU9250_WriteReg(MPU9250_ADDR, MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    MPU9250_WriteReg(MPU9250_ADDR, MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU9250_WriteReg(MPU9250_ADDR, MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU9250_WriteReg(MPU9250_ADDR, MPU_INTBP_CFG_REG, 0X82); // INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    res = MPU9250_ReadReg(MPU9250_ADDR, MPU_DEVICE_ID_REG);  // 读取MPU6500的ID
    if (res == MPU6500_ID)                                 // 器件ID正确
    {
        MPU9250_WriteReg(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU9250_WriteReg(MPU9250_ADDR, MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
        MPU_Set_Rate(50);                                      // 设置采样率为50Hz
    } else
        return 1;

    res = MPU9250_ReadReg(AK8963_ADDR, MAG_WIA); // 读取AK8963 ID
    if (res == AK8963_ID) {
        MPU9250_WriteReg(AK8963_ADDR, MAG_CNTL1, 0X11); // 设置AK8963为单次测量模式
    } else
        return 1;

    return 0;
}

// 设置MPU9250陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU9250_WriteReg(MPU9250_ADDR, MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}
// 设置MPU9250加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU9250_WriteReg(MPU9250_ADDR, MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}

// 设置MPU9250的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU9250_WriteReg(MPU9250_ADDR, MPU_CFG_REG, data); // 设置数字低通滤波器
}

// 设置MPU9250的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000) rate = 1000;
    if (rate < 4) rate = 4;
    data = 1000 / rate - 1;
    data = MPU9250_WriteReg(MPU9250_ADDR, MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
    return MPU_Set_LPF(rate / 2);                                   // 自动设置LPF为采样率的一半
}

// 得到温度值
// 返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU9250_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw  = ((u16)buf[0] << 8) | buf[1];
    temp = 21 + ((double)raw) / 333.87;
    return temp * 100;
}
// 得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
void MPU_Get_Gyroscope(float *gx, float *gy, float *gz)
{
    int16_t x, y, z;
    MPU9250_DATA.Data_H = MPU9250_ReadReg(MPU9250_ADDR,MPU_GYRO_XOUTH_REG);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(MPU9250_ADDR,MPU_GYRO_XOUTL_REG);
    x = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    MPU9250_DATA.Data_H = MPU9250_ReadReg(MPU9250_ADDR,MPU_GYRO_YOUTH_REG);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(MPU9250_ADDR,MPU_GYRO_YOUTL_REG);
    y = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    MPU9250_DATA.Data_H = MPU9250_ReadReg(MPU9250_ADDR,MPU_GYRO_ZOUTH_REG);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(MPU9250_ADDR,MPU_GYRO_ZOUTL_REG);
    z = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    *gx = x * 2000.0 / 32768.0;
    *gy = y * 2000.0 / 32768.0;
    *gz = z * 2000.0 / 32768.0;
}
// 得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
void MPU_Get_Accelerometer(float *ax, float *ay, float *az)
{
    int16_t x, y, z;
    MPU9250_DATA.Data_H = MPU9250_ReadReg(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(MPU9250_ADDR,MPU_ACCEL_XOUTL_REG);
    x = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    MPU9250_DATA.Data_H = MPU9250_ReadReg(MPU9250_ADDR,MPU_ACCEL_YOUTH_REG);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(MPU9250_ADDR,MPU_ACCEL_YOUTL_REG);
    y = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    MPU9250_DATA.Data_H = MPU9250_ReadReg(MPU9250_ADDR,MPU_ACCEL_ZOUTH_REG);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(MPU9250_ADDR,MPU_ACCEL_ZOUTL_REG);
    z = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    *ax = x * 2 * g / 32768.0;
    *ay = y * 2 * g / 32768.0;
    *az = z * 2 * g / 32768.0;
}

// 得到磁力计值(原始值)
// mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
void MPU_Get_Magnetometer(float *mx, float *my, float *mz)
{
    int16_t x, y, z;
    int8_t ax,ay,az;
    MPU9250_DATA.Data_H = MPU9250_ReadReg(AK8963_ADDR,MAG_XOUT_H);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(AK8963_ADDR,MAG_XOUT_L);
    x = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    MPU9250_DATA.Data_H = MPU9250_ReadReg(AK8963_ADDR, MAG_YOUT_H);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(AK8963_ADDR, MAG_YOUT_L);
    y = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    MPU9250_DATA.Data_H = MPU9250_ReadReg(AK8963_ADDR, MAG_ZOUT_H);
    MPU9250_DATA.Data_L = MPU9250_ReadReg(AK8963_ADDR, MAG_ZOUT_L);
    z = (int16_t)((MPU9250_DATA.Data_H << 8) | MPU9250_DATA.Data_L);
    MPU9250_DATA.Data_H = MPU9250_ReadReg(AK8963_ADDR, 0x10);
    ax = (int8_t)MPU9250_DATA.Data_H;
    MPU9250_DATA.Data_H = MPU9250_ReadReg(AK8963_ADDR, 0x11);
    ay = (int8_t)MPU9250_DATA.Data_H;
    MPU9250_DATA.Data_H = MPU9250_ReadReg(AK8963_ADDR, 0x12);
    az = (int8_t)MPU9250_DATA.Data_H;
    *mx = ((ax - 128) * 0.5f /128 +1) * x * 0.15f;
    *my = ((ay - 128) * 0.5f /128 +1) * y * 0.15f;
    *mz = ((az - 128) * 0.5f /128 +1) * z * 0.15f;
    MPU9250_WriteReg(AK8963_ADDR, MAG_CNTL1, 0X11);
}

void MPU_Offset(void)
{
    u8 i = 0;
    float ax,ay,az,gx,gy,gz;
    for(i = 0; i < 100; i++)
    {
        MPU_Get_Gyroscope(&MPU9250_DATA.GYRO_XOUT,&MPU9250_DATA.GYRO_YOUT,&MPU9250_DATA.GYRO_ZOUT);			 //读取角速度原始数据
		MPU_Get_Accelerometer(&MPU9250_DATA.ACCEL_XOUT,&MPU9250_DATA.ACCEL_YOUT,&MPU9250_DATA.ACCEL_ZOUT);
        ax += MPU9250_DATA.ACCEL_XOUT;
        ay += MPU9250_DATA.ACCEL_YOUT;
        az += MPU9250_DATA.ACCEL_ZOUT;
        gx += MPU9250_DATA.GYRO_XOUT;
        gy += MPU9250_DATA.GYRO_YOUT;
        gz += MPU9250_DATA.GYRO_ZOUT;
        delay_ms(10);
    }
    MPU9250_DATA.ACCEL_XOUT_OFFSET = (float)(ax/100);
    MPU9250_DATA.ACCEL_YOUT_OFFSET = (float)(ay/100);
    MPU9250_DATA.ACCEL_ZOUT_OFFSET = (float)(az/100-9.8f);
    MPU9250_DATA.GYRO_XOUT_OFFSET = (float)(gx/100);
    MPU9250_DATA.GYRO_YOUT_OFFSET = (float)(gy/100);
    MPU9250_DATA.GYRO_ZOUT_OFFSET = (float)(gz/100);
    MPU9250_DATA.MAP_XOUT_OFFSET = 6.075f;
    MPU9250_DATA.MAP_YOUT_OFFSET = -0.53f;
    MPU9250_DATA.MAP_ZOUT_OFFSET = 2.33f;
}

void MPU_GetData(void)
{
    MPU_Get_Gyroscope(&MPU9250_DATA.GYRO_XOUT,&MPU9250_DATA.GYRO_YOUT,&MPU9250_DATA.GYRO_ZOUT);			 //读取角速度原始数据
    MPU_Get_Accelerometer(&MPU9250_DATA.ACCEL_XOUT,&MPU9250_DATA.ACCEL_YOUT,&MPU9250_DATA.ACCEL_ZOUT);
    MPU_Get_Magnetometer(&MPU9250_DATA.MAP_XOUT,&MPU9250_DATA.MAP_YOUT,&MPU9250_DATA.MAP_ZOUT);
    MPU9250_DATA.ACCEL_XOUT -= MPU9250_DATA.ACCEL_XOUT_OFFSET;
    MPU9250_DATA.ACCEL_YOUT -= MPU9250_DATA.ACCEL_YOUT_OFFSET;
    MPU9250_DATA.ACCEL_ZOUT -= MPU9250_DATA.ACCEL_ZOUT_OFFSET;
    MPU9250_DATA.GYRO_XOUT -= MPU9250_DATA.GYRO_XOUT_OFFSET;
    MPU9250_DATA.GYRO_YOUT -= MPU9250_DATA.GYRO_YOUT_OFFSET;
    MPU9250_DATA.GYRO_ZOUT -= MPU9250_DATA.GYRO_ZOUT_OFFSET;
    MPU9250_DATA.MAP_XOUT -= MPU9250_DATA.MAP_XOUT_OFFSET;
    MPU9250_DATA.MAP_YOUT -= MPU9250_DATA.MAP_YOUT_OFFSET;
    MPU9250_DATA.MAP_ZOUT -= MPU9250_DATA.MAP_ZOUT_OFFSET;
}

void find_MAX(void)
{
    float mx_max=0, mx_min=0, my_max=0, my_min=0, mz_max=0, mz_min=0;
    while(1)
    {
        MPU_Get_Magnetometer(&MPU9250_DATA.MAP_XOUT,&MPU9250_DATA.MAP_YOUT,&MPU9250_DATA.MAP_ZOUT);
        if(MPU9250_DATA.MAP_XOUT > mx_max)
        {
            mx_max = MPU9250_DATA.MAP_XOUT;
        }
        if(MPU9250_DATA.MAP_XOUT < mx_min)
        {
            mx_min = MPU9250_DATA.MAP_XOUT;
        }
        if(MPU9250_DATA.MAP_YOUT > my_max)
        {
            my_max = MPU9250_DATA.MAP_YOUT;
        }
        if(MPU9250_DATA.MAP_YOUT < my_min)
        {
            my_min = MPU9250_DATA.MAP_YOUT;
        }
        if(MPU9250_DATA.MAP_ZOUT > mz_max)
        {
            mz_max = MPU9250_DATA.MAP_ZOUT;
        }
        if(MPU9250_DATA.MAP_ZOUT < mz_min)
        {
            mz_min = MPU9250_DATA.MAP_ZOUT;
        }
        printf("mx_Max:%0.2f,my_Max:%0.2f,mz_Max:%0.2f\r\n",mx_max,my_max,mz_max);
        printf("mx_Min:%0.2f,my_Min:%0.2f,mz_Min:%0.2f\r\n",mx_min,my_min,mz_min);
    }
}