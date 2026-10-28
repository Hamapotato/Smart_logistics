#include "tubes.h"

unsigned char tab[] = {
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
    0x77, /* A */
    0x7C, /* B */
    0x39, /* C */
    0x5E, /* D */
    0x79, /* E */
    0x71, /* F */
    0x76, /* G */
    0x00, /* 黑屏 */
};

// 存放四位数码管的显示数据
unsigned char disp_num[] = {0x3F, 0x3F, 0x3F, 0x3F};
// [红外启动] 数码管当前显示的数字，默认 2（维持原来 0002 的画面）
u16 TUBES_VALUE = 2;
// 数码管端口初始化
void TUBES_Driver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
}

void TUBES_DIO_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void TUBES_DIO_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 发送起始信号
void TUBES_Start(void)
{
    TUBES_DIO_OUT();
    TUBES_W_CLK = 1;
    TUBES_W_DIO = 1;
    delay_us(2);
    TUBES_W_DIO = 0;
}
// 发送结束信号
void TUBES_Stop(void)
{
    TUBES_DIO_OUT();
    TUBES_W_CLK=0;
    delay_us(2);
    TUBES_W_DIO=0;
    delay_us(2);
    TUBES_W_CLK=1;
    delay_us(2);
    TUBES_W_DIO=1;
}
// 等待应答
u8 TUBES_Ack(void)
{
    u8 i = 0;
    TUBES_DIO_OUT();
    TUBES_W_CLK=0;
    delay_us(5);
    TUBES_DIO_IN();
    while (TUBES_R_DIO) {
        i++;
        if (i > 250) {
            TUBES_Stop();
            return 1;
        }
    }
    TUBES_W_CLK=1;
    delay_us(2);
    TUBES_W_CLK=0;
    return 0;
}
// 写一个字节数学
void TUBES_WriteByte(unsigned char oneByte)
{
    unsigned char i;
    TUBES_DIO_OUT();
    for (i = 0; i < 8; i++) {
        TUBES_W_CLK=0;
        if( oneByte & 0x01 )									//低位在前
        {
            TUBES_W_DIO = 1;
        }
        else
        {
            TUBES_W_DIO = 0;
        }
        delay_us( 3 );
        oneByte = oneByte >> 1;
        TUBES_W_CLK = 1;
        delay_us( 3 );
    }
}
static void TUBES_WriteDisplay(void)
{
    unsigned char i;
    TUBES_Start();
    TUBES_WriteByte(0x40); // 地址自增模式
    TUBES_Ack();
    TUBES_Stop();
    TUBES_Start();
    TUBES_WriteByte(0xC0);
    TUBES_Ack();
    for (i = 0; i < 4; i++) {
        TUBES_WriteByte(disp_num[i]);
        TUBES_Ack();
    }
    TUBES_Stop();
}

// 显示数字
void TUBES_Display(void)
{
    disp_num[0] = tab[(TUBES_VALUE / 1000) % 10];
    disp_num[1] = tab[(TUBES_VALUE / 100) % 10];
    disp_num[2] = tab[(TUBES_VALUE / 10) % 10];
    disp_num[3] = tab[TUBES_VALUE % 10];
    TUBES_WriteDisplay();
}

// digits[i] = 0..9 显示数字；其他值显示为空白
void TUBES_DisplayDigits(const unsigned char digits[4])
{
    unsigned char i;
    for (i = 0; i < 4; i++) {
        disp_num[i] = (digits[i] <= 9) ? tab[digits[i]] : tab[17];
    }
    TUBES_WriteDisplay();
}

// 四位数码管全部熄灭
void TUBES_Clear(void)
{
    static const unsigned char blank[4] = {10, 10, 10, 10};
    TUBES_DisplayDigits(blank);
}
// [红外启动] 设置 TUBES_Display() 要显示的数字（0-9999）
void TUBES_SetValue(u16 value)
{
    TUBES_VALUE = (u16)(value % 10000);
}
// 设置亮度：0-7
void TUBES_SetBrightness(unsigned char level)
{
    TUBES_Start();
    TUBES_WriteByte(0x88 | level);
    TUBES_Ack();
    TUBES_Stop();
}