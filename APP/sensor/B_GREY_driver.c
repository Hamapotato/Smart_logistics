#include "GREY_driver.h"

#define B_SW_I2C1_PIN_SCL GPIO_Pin_1
#define B_SW_I2C1_PIN_SDA GPIO_Pin_2
#define B_SW_I2C1_PIN   GPIOD
#define B_SW_I2C1_CLOCK RCC_AHB1Periph_GPIOD

static void b_sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface);
static void b_sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface);

static void b_sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit);
static uint8_t b_sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface);

static uint8_t b_sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t byte);
static uint8_t b_sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack);

/**
 * 初始化i2c
 */
void b_sw_i2c_init(void)
{
    RCC_AHB1PeriphClockCmd(B_SW_I2C1_CLOCK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin   = B_SW_I2C1_PIN_SCL | B_SW_I2C1_PIN_SDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(B_SW_I2C1_PIN, &GPIO_InitStructure);

    GPIO_SetBits(B_SW_I2C1_PIN, B_SW_I2C1_PIN_SCL | B_SW_I2C1_PIN_SDA);
}

int8_t b_sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data, uint8_t data_length)
{
    uint8_t i;
    uint8_t ack_bit;

    /* 起始位 */
    b_sw_i2c_hal_start(i2c_interface);

    /* 地址+读写位 */
    ack_bit = b_sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_READ);
    if (ack_bit) {
        /* 从设备没有回复ACK,直接退出 */
        b_sw_i2c_hal_stop(i2c_interface);
        return 1;
    }

    /* 连续读取N-1个数据 给ACK */
    for (i = 0; i < data_length - 1; ++i) {
        data[i] = b_sw_i2c_hal_read_byte(i2c_interface, ACK);
    }

    /* 最后一个数据给 NACK */
    data[i] = b_sw_i2c_hal_read_byte(i2c_interface, NACK);

    /* 停止位 */
    b_sw_i2c_hal_stop(i2c_interface);
    return 0;
}

int8_t b_sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length)
{
    uint8_t i;
    uint8_t ack_bit;

    /* 起始位 */
    b_sw_i2c_hal_start(i2c_interface);

    /* 地址+读写位 */
    ack_bit = b_sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
    if (ack_bit) {
        /* 从设备没有回复ACK,直接退出 */
        b_sw_i2c_hal_stop(i2c_interface);
        return 1;
    }

    /* 连续写入N个数据, 每次读取1 bit的 ACK */
    for (i = 0; i < data_length; ++i) {
        ack_bit = b_sw_i2c_hal_write_byte(i2c_interface, data[i]);
    }

    /* 停止位 */
    b_sw_i2c_hal_stop(i2c_interface);
    return 0;
}

int8_t b_sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data)
{
    return b_sw_i2c_read(i2c_interface, dev_addr, data, 1);
}

int8_t b_sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t data)
{
    return b_sw_i2c_write(i2c_interface, dev_addr, &data, 1);
}

int8_t b_sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface,
                         uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length)
{
    uint8_t ack_bit;
    b_sw_i2c_hal_start(i2c_interface);
    ack_bit = b_sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
    if (ack_bit) {
        b_sw_i2c_hal_stop(i2c_interface);
        return 1;
    }
    ack_bit = b_sw_i2c_hal_write_byte(i2c_interface, mem_addr);

    return b_sw_i2c_read(i2c_interface, dev_addr, data, data_length);
}

int8_t b_sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data,
                          uint8_t data_length)
{
    uint8_t ack_bit;
    b_sw_i2c_hal_start(i2c_interface);
    ack_bit = b_sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
    if (ack_bit) {
        b_sw_i2c_hal_stop(i2c_interface);
        return 1;
    }
    ack_bit = b_sw_i2c_hal_write_byte(i2c_interface, mem_addr);

    return b_sw_i2c_write(i2c_interface, dev_addr, data, data_length);
}

/***************************
 * 基礎操作抽象层
 **************************/

/**
 * @brief send start bit by driving sda and scl LOW
 * @param i2c_interface
 */
static void b_sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface)
{
    i2c_interface->sda_out(HIGH, i2c_interface->user_data);
    i2c_interface->scl_out(HIGH, i2c_interface->user_data);
    i2c_interface->sda_out(LOW, i2c_interface->user_data);
    i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief send stop bit
 * @param i2c_interface
 */
static void b_sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface)
{
    i2c_interface->sda_out(LOW, i2c_interface->user_data);
    i2c_interface->scl_out(HIGH, i2c_interface->user_data);
    i2c_interface->sda_out(HIGH, i2c_interface->user_data);
}

/**
 * @brief 输出 sda 电平,然后 scl 输出一个时钟
 * @param i2c_interface
 * @param bit bit level to send, 0:LOW, 1:HIGH
 */
static void b_sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit)
{
    i2c_interface->sda_out(bit, i2c_interface->user_data);
    i2c_interface->scl_out(HIGH, i2c_interface->user_data);
    i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief 读 sda 电平值,然后 scl 输出一个时钟
 * @param i2c_interface
 * @return 返回 SDA 电平值, 0:LOW, 1:HIGH
 */
static uint8_t b_sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface)
{
    uint8_t bit;
    i2c_interface->sda_out(HIGH, i2c_interface->user_data);
    i2c_interface->scl_out(HIGH, i2c_interface->user_data);
    bit = i2c_interface->sda_in(i2c_interface->user_data);
    i2c_interface->scl_out(LOW, i2c_interface->user_data);
    return bit;
}

/**
 * @brief 向IIC输出一个字节
 * @param i2c_interface
 * @param byte
 * @return 从设备反馈的 ACK 电平值
 */
static uint8_t b_sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t byte)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; ++i) {
        b_sw_i2c_hal_write_bit(i2c_interface, byte & (0x80 >> i));
    }

    ack = b_sw_i2c_hal_read_bit(i2c_interface);
    return ack;
}

/**
 * @brief 从IIC总线上读取一个字节
 * @param i2c_interface
 * @param ack 向从设备反馈 ACK 或者 NACK
 * @return 读取到的字节
 */
static uint8_t b_sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack)
{
    uint8_t byte = 0;
    uint8_t i;

    i2c_interface->sda_out(HIGH, i2c_interface->user_data);
    for (i = 0; i < 8; ++i) {
        i2c_interface->scl_out(HIGH, i2c_interface->user_data);
        byte <<= 1;
        byte |= i2c_interface->sda_in(i2c_interface->user_data);
        i2c_interface->scl_out(LOW, i2c_interface->user_data);
    }

    b_sw_i2c_hal_write_bit(i2c_interface, ack);
    return byte;
}

/* 定义sda输出函数 bit=0为低电平 bit=1为高电平 */
void b_sda_out(uint8_t bit, void *user_data)
{
    GPIO_WriteBit(B_SW_I2C1_PIN, B_SW_I2C1_PIN_SDA, (BitAction)bit);

    /* IIC软件延迟 */
    delay_us(10);
}

/* 定义sda读取函数 bit 为返回的电平值 */
uint8_t b_sda_in(void *user_data)
{
    uint8_t bit;
    bit = (uint8_t)GPIO_ReadInputDataBit(B_SW_I2C1_PIN, B_SW_I2C1_PIN_SDA);

    /* IIC软件延迟 */
    delay_us(10);
    return bit;
}

/* 定义scl时钟输出函数 bit=0为低电平 bit=1为高电平 */
void b_scl_out(uint8_t bit, void *user_data)
{
    GPIO_WriteBit(B_SW_I2C1_PIN, B_SW_I2C1_PIN_SCL, (BitAction)bit);

    /* IIC软件延迟 */
    delay_us(10);
}

/**
 * i2c地址扫描
 * @param scan_addr 扫描出来的地址存放,数值不为0的为扫描到的地址，扫到的地址会挨个放在数组的最前面
 * @return 返回扫描到的设备数量, 0为无设备发现
 */
uint8_t b_i2c_scan(sw_i2c_interface_t *i2c_interface, uint8_t *scan_addr)
{
    int i;
    uint8_t count = 0;
    uint8_t data;
    int8_t ret;

    for (i = 1; i < 127; ++i) {
        ret = b_sw_i2c_read(i2c_interface, i << 1, &data, 1);
        if (ret == 0) {
            scan_addr[count] = i;
            ++count;
        }
    }

    return count;
}

/* 设置软件IIC驱动 */
sw_i2c_interface_t b_i2c_interface =
    {
        .sda_in    = b_sda_in,
        .scl_out   = b_scl_out,
        .sda_out   = b_sda_out,
        .user_data = 0, // 用户数据，可在输入输出函数里得到
};

void B_GW_digital_Init(void) // 0000 0000
{
    /* 存放扫描到的地址 */
    uint8_t ping_response;
    uint8_t scan_addr[128] = {0};
    volatile uint8_t count;
    /* 初始化IIC */
    b_sw_i2c_init();
    /* ps: 软件IIC初始化的时候会出发一次IIC start，会导致第一次IIC通讯会失败 */

    /* 第一次IIC通讯会失败（因为软件IIC触发了start），手动发个stop也能消除 */
    b_sw_i2c_mem_read(&b_i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_PING, &ping_response, 1);
    /* 后面IIC通讯是正常的 */
    b_sw_i2c_mem_read(&b_i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_PING, &ping_response, 1);

    /* 扫描开始 */
    count = b_i2c_scan(&b_i2c_interface, scan_addr);

    /* 打开开关量数据模式 */
    b_sw_i2c_write_byte(&b_i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_DIGITAL_MODE);
}

uint8_t B_GW_digital_Read(void)
{
    uint8_t digital_data;
    /* 读取开关量数据 */
    b_sw_i2c_read_byte(&b_i2c_interface, GW_GRAY_ADDR_DEF << 1, &digital_data); // digital_data 有1~8号探头开关数据
    return digital_data;
} // 0000 0000

int16_t B_GW_digital_eight(void)
{
	uint8_t digital_data;
	uint8_t gray_sensor[8];
	
	digital_data = reverse(~B_GW_digital_Read());
	/* 把字节里的8个开关量存到八个变量里，这里为gray_sensor[0] ~ gray_sensor[7], 
		 * 也可以是变量val1 ~ val8, 因为是宏定义 */
	SEP_ALL_BIT8(digital_data, 
		gray_sensor[0], //探头1
		gray_sensor[1], //探头2
		gray_sensor[2], //探头3
		gray_sensor[3], //探头4
		gray_sensor[4], //探头5
		gray_sensor[5], //探头6
		gray_sensor[6], //探头7
		gray_sensor[7]  //探头8
	);
	return gray_sensor[0]*6+gray_sensor[1]*4+gray_sensor[2]*2+gray_sensor[3]*1+gray_sensor[4]*(-1)+gray_sensor[5]*(-2)+gray_sensor[6]*(-4)+gray_sensor[7]*(-6);
}