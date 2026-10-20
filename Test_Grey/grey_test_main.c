/* ============================================================================
 *  八路巡线传感器测试固件 —— 独立工程 Test_Grey
 *
 *  【被测对象】感为科技（GW）8 路 I2C 灰度巡线模块 ×4，各占一条软件 I2C 总线：
 *      F 前   PB13 = SCL   PB14 = SDA
 *      B 后   PD1  = SCL   PD2  = SDA
 *      L 左   PC8  = SCL   PC9  = SDA
 *      R 右   PC3  = SCL   PC2  = SDA      （注意这一路 SCL 比 SDA 号大）
 *    模块 I2C 地址固定 GW_GRAY_ADDR_DEF = 0x4F（左移一位发到总线是 0x9E）。
 *    PING 命令 0xAA，正确应答 0x66；0xDD 切数字模式；0xC1 读固件版本。
 *
 *  【安全】本固件只读取和切模式，绝不发送会改模块内部参数的命令：
 *      0xD0 / 0xD1 （黑/白标定）
 *      0xC0        （重启）
 *      0xAD        （改 I2C 地址）
 *    免得破坏比赛固件正在用的模块标定值。
 *
 *  【输出】
 *    串口 USART1 19200 8N1（PA9 = TX，PA10 = RX）—— 主要通道，逐路明细
 *    数码管 TM1650                               —— 显示 iNNN：i = 模块号 1..4，
 *                                                   NNN = 该模块原始字节的十进制
 *    LED_GREEN                                   —— 心跳，每轮翻转
 *    LED_RED                                     —— 有模块自检没过时点亮
 *
 *  判读方法见同目录《巡线传感器测试流程.md》。
 * ========================================================================== */
#include "main.h"

/* ---------------------------------------------------------------------------
 * 底层软 I2C 原语在驱动里都是非 static 的，但 GREY_driver.h 没声明它们，
 * 这里补外部声明。（x_GW_digital_Init / x_GW_digital_Read 头里已有。）
 * --------------------------------------------------------------------------- */
extern sw_i2c_interface_t f_i2c_interface;
extern int8_t  f_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t  f_sw_i2c_write_byte(sw_i2c_interface_t *, uint8_t, const uint8_t);
extern uint8_t f_i2c_scan(sw_i2c_interface_t *, uint8_t *);

extern sw_i2c_interface_t b_i2c_interface;
extern int8_t  b_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t  b_sw_i2c_write_byte(sw_i2c_interface_t *, uint8_t, const uint8_t);
extern uint8_t b_i2c_scan(sw_i2c_interface_t *, uint8_t *);

extern sw_i2c_interface_t l_i2c_interface;
extern int8_t  l_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t  l_sw_i2c_write_byte(sw_i2c_interface_t *, uint8_t, const uint8_t);
extern uint8_t l_i2c_scan(sw_i2c_interface_t *, uint8_t *);

extern sw_i2c_interface_t r_i2c_interface;
extern int8_t  r_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t  r_sw_i2c_write_byte(sw_i2c_interface_t *, uint8_t, const uint8_t);
extern uint8_t r_i2c_scan(sw_i2c_interface_t *, uint8_t *);

/* 左移一位后的 7 位地址，即真正发到总线上的形式 */
#define GW_ADDR8 ((uint8_t)(GW_GRAY_ADDR_DEF << 1))

/* 循环 500ms；每 2 轮换一个数码管显示的模块；每 10/20 轮打一次统计/模拟量 */
#define CYCLE_MS     500
#define TUBE_EVERY   2
#define STATS_EVERY  10
#define ANALOG_EVERY 20

/* 逐路模拟量是实验性的：本工程既有代码从没用过 0xB0+n，所以留个开关。
 * 万一模块不认这些寄存器，置 0 关掉即可，其它功能不受影响。 */
#define GREY_TEST_ANALOG 1

/* --------------------------------------------------------------------------- */

typedef struct {
    char        tag;        /* F / B / L / R */
    const char *bus;        /* 总线引脚，已补齐宽度，串口里好对齐 */
    sw_i2c_interface_t *iface;
    int8_t  (*mem_read)(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
    int8_t  (*write_cmd)(sw_i2c_interface_t *, uint8_t, const uint8_t);
    uint8_t (*scan)(sw_i2c_interface_t *, uint8_t *);
    void    (*init)(void);
    uint8_t (*read)(void);
} GREY_DEV;

static const GREY_DEV DEV[4] = {
    { 'F', "PB13/14 ", &f_i2c_interface, f_sw_i2c_mem_read, f_sw_i2c_write_byte, f_i2c_scan, F_GW_digital_Init, F_GW_digital_Read },
    { 'B', "PD1/2   ", &b_i2c_interface, b_sw_i2c_mem_read, b_sw_i2c_write_byte, b_i2c_scan, B_GW_digital_Init, B_GW_digital_Read },
    { 'L', "PC8/9   ", &l_i2c_interface, l_sw_i2c_mem_read, l_sw_i2c_write_byte, l_i2c_scan, L_GW_digital_Init, L_GW_digital_Read },
    { 'R', "PC3/2   ", &r_i2c_interface, r_sw_i2c_mem_read, r_sw_i2c_write_byte, r_i2c_scan, R_GW_digital_Init, R_GW_digital_Read },
};

/* 自检结果 */
static u8 PING_OK[4];    /* 1 = 该模块应答了 0x66 */
static u8 SCAN_CNT[4];   /* 该总线上扫到几个设备 */

/* 运行期统计：用来找“自开机以来从未翻转”的那一路 */
static u8  PREV[4];        /* 上一轮原始字节 */
static u8  PREV_VALID[4];  /* 上一轮是否有效（第一轮不算翻转） */
static u8  EVER_HIGH[4][8];
static u8  EVER_LOW[4][8];
static u16 CHG[4][8];      /* 每路翻转次数 */

/* ---------------------------------------------------------------------------
 * 极简格式化：本工程用 MicroLIB，格式串只用 %s，数字自己拼进缓冲区。
 * 这样不依赖 MicroLIB 对宽度/标志（%02X、%-9s…）的支持程度。
 * --------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------
 * reverse() 定义在 APP\trace\trace.c，那是比赛固件才编的文件，本工程没有它。
 * 可是四个巡线驱动里的 x_GW_digital_eight() 都引用了 reverse，链接时需要。
 * 本测试程序不调用 x_GW_digital_eight()，这里补一个同样的实现即可。
 * （trace.h 已经声明过，trace.c 不在本工程，不会重复定义。）
 * --------------------------------------------------------------------------- */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}

static char *p_str(char *o, const char *s)
{
    while (*s) *o++ = *s++;
    return o;
}

static char *p_u16(char *o, u16 v)
{
    char t[6];
    u8   n = 0;
    if (v == 0) { *o++ = '0'; return o; }
    while (v) { t[n++] = (char)('0' + (v % 10)); v = (u16)(v / 10); }
    while (n) *o++ = t[--n];
    return o;
}

/* 固定 3 位十进制，补前导 0 —— 正好对应数码管的一位 */
static char *p_dec3(char *o, u8 v)
{
    *o++ = (char)('0' + (v / 100) % 10);
    *o++ = (char)('0' + (v / 10) % 10);
    *o++ = (char)('0' + v % 10);
    return o;
}

static char *p_hex2(char *o, u8 v)
{
    const char *H = "0123456789ABCDEF";
    *o++ = H[(v >> 4) & 0x0F];
    *o++ = H[v & 0x0F];
    return o;
}

/* 探头 1..8 = bit0..bit7，按探头顺序打印 */
static char *p_bits(char *o, u8 raw)
{
    u8 b;
    for (b = 0; b < 8; b++) {
        *o++ = (raw & (1u << b)) ? '1' : '0';
        if (b != 7) *o++ = ' ';
    }
    return o;
}

/* 行首标签：[ 12] 定宽，方便串口里对齐 */
static char *p_label(char *o, u16 n)
{
    *o++ = '[';
    if (n < 100) *o++ = ' ';
    if (n < 10)  *o++ = ' ';
    o = p_u16(o, n);
    *o++ = ']';
    *o++ = ' ';
    return o;
}

static char *p_head(char *o, u8 i)
{
    *o++ = ' ';
    *o++ = DEV[i].tag;
    *o++ = ' ';
    return p_str(o, DEV[i].bus);
}

/* 收尾：在写入位置打结束符，整行一次 printf 出去 */
static void flush(char *buf, char *o)
{
    *o = '\0';
    printf("%s\r\n", buf);
}

/* ---------------------------------------------------------------------------
 * 自检：把通信层一次查清楚
 * --------------------------------------------------------------------------- */
static void GreyTest_Selftest(void)
{
    u8 i, k;
    u8 ok = 0;

    printf("\r\n");
    printf("==============================================\r\n");
    printf(" 巡线传感器测试固件   GW 8路 I2C   地址 0x4F\r\n");
    printf(" 串口 19200 8N1    数码管 iNNN = 模块i的原始字节\r\n");
    printf(" 打印顺序 = 探头1..8，对应原始字节 bit0..bit7\r\n");
    printf("==============================================\r\n");
    printf("[自检]\r\n");

    for (i = 0; i < 4; i++) {
        char  buf[128];
        char *o = buf;
        u8    rsp = 0;
        u8    fw  = 0;
        u8    sa[16];
        int8_t e1, e2;

        /* 先用驱动自己的 Init：内部做 软I2C初始化 + 丢一次通讯 + 扫描 + 切数字模式。
         * 这是比赛固件一直在跑的代码，等于顺带验证了初始化路径本身。 */
        DEV[i].init();

        /* 再自己问一次 PING，好把结果打出来 */
        DEV[i].mem_read(DEV[i].iface, GW_ADDR8, GW_GRAY_PING, &rsp, 1); /* 丢掉 */
        rsp = 0;
        e1  = DEV[i].mem_read(DEV[i].iface, GW_ADDR8, GW_GRAY_PING, &rsp, 1);
        e2  = DEV[i].mem_read(DEV[i].iface, GW_ADDR8, GW_GRAY_FIRMWARE, &fw, 1);
        SCAN_CNT[i] = DEV[i].scan(DEV[i].iface, sa);

        PING_OK[i] = (u8)((e1 == 0) && (rsp == GW_GRAY_PING_OK));
        if (PING_OK[i]) ok++;

        o = p_head(o, i);
        o = p_str(o, PING_OK[i] ? "PING=OK   " : "PING=FAIL ");
        o = p_str(o, "rsp=0x");
        o = p_hex2(o, rsp);
        o = p_str(o, "  scan=");
        o = p_u16(o, SCAN_CNT[i]);
        if (SCAN_CNT[i]) {
            o = p_str(o, " [");
            for (k = 0; k < SCAN_CNT[i] && k < 16; k++) {
                o = p_str(o, "0x");
                o = p_hex2(o, sa[k]);
                if (k + 1 < SCAN_CNT[i]) *o++ = ',';
            }
            *o++ = ']';
        }
        o = p_str(o, "  fw=0x");
        o = p_hex2(o, fw);
        if (e2 != 0) o = p_str(o, "(读失败)");
        flush(buf, o);
    }

    {
        char  buf[96];
        char *o = buf;
        o = p_str(o, "[自检结论] PING 通过 ");
        o = p_u16(o, ok);
        o = p_str(o, "/4   ");
        o = p_str(o, (ok == 4) ? "四台模块通讯都正常"
                               : "看上面 FAIL 的那几台");
        flush(buf, o);
    }
    printf("\r\n");
}

/* ---------------------------------------------------------------------------
 * 一轮：读四台，打印原始字节 + 8 路电平，并累计翻转次数
 * --------------------------------------------------------------------------- */
static void GreyTest_Cycle(u16 n)
{
    u8 i, b;

    for (i = 0; i < 4; i++) {
        char  buf[128];
        char *o = buf;
        u8    raw;

        raw = DEV[i].read();

        for (b = 0; b < 8; b++) {
            if (raw & (1u << b)) EVER_HIGH[i][b] = 1;
            else                 EVER_LOW[i][b]  = 1;
            if (PREV_VALID[i] && ((PREV[i] ^ raw) & (1u << b))) {
                if (CHG[i][b] < 60000) CHG[i][b]++;
            }
        }
        PREV[i]       = raw;
        PREV_VALID[i] = 1;

        if (i == 0) o = p_label(o, n);
        else        o = p_str(o, "      ");
        o = p_head(o, i);
        o = p_str(o, "raw=0x");
        o = p_hex2(o, raw);
        o = p_str(o, "  P1..P8= ");
        o = p_bits(o, raw);
        flush(buf, o);

        /* 数码管轮换显示某一台的 iNNN */
        if ((n / TUBE_EVERY) % 4 == i) {
            TUBES_SetValue((u16)((i + 1) * 1000 + raw));
        }
    }
}

/* 累计统计：一眼看出哪一路从没动过 */
static void GreyTest_Stats(void)
{
    u8 i, b;

    printf("\r\n--- 翻转统计（自开机累计）  0 = 该路从未翻转，重点怀疑 ---\r\n");
    for (i = 0; i < 4; i++) {
        char  buf[128];
        char *o = buf;
        o = p_str(o, " ");
        o = p_head(o, i);
        o = p_str(o, "chg ");
        for (b = 0; b < 8; b++) {
            o = p_u16(o, CHG[i][b]);
            if (b != 7) *o++ = ' ';
        }
        flush(buf, o);
    }
    printf("（0 表示这一路从开机到现在电平一次都没变过）\r\n\r\n");
}

#if GREY_TEST_ANALOG
/* 实验性：切模拟模式，逐路读 0xB0+1 .. 0xB0+8，读完立刻切回数字模式 */
static void GreyTest_Analog(void)
{
    u8 i, k;

    printf("\r\n--- 模拟量快照 0xB0+n（实验性，读完立刻切回数字模式）---\r\n");
    for (i = 0; i < 4; i++) {
        char  buf[160];
        char *o = buf;
        u8    v;

        o = p_head(o, i);

        /* 0xB0 = GW_GRAY_ANALOG_MODE */
        if (DEV[i].write_cmd(DEV[i].iface, GW_ADDR8, GW_GRAY_ANALOG_MODE) != 0) {
            o = p_str(o, "切模拟模式 NACK");
            flush(buf, o);
            continue;
        }
        for (k = 1; k <= 8; k++) {
            if (DEV[i].mem_read(DEV[i].iface, GW_ADDR8,
                                (uint8_t)GW_GRAY_ANALOG(k), &v, 1) != 0) {
                o = p_str(o, "NACK");
            } else {
                o = p_dec3(o, v);
            }
            if (k != 8) *o++ = ' ';
        }

        /* 不管读没读成，都要切回数字模式，别把模块留在模拟模式 */
        DEV[i].write_cmd(DEV[i].iface, GW_ADDR8, GW_GRAY_DIGITAL_MODE);

        flush(buf, o);
    }
    printf("（值域 0..255；某一路恒为 0 或恒为 255 且不随黑线变化 = 可能坏了）\r\n\r\n");
}
#endif

/* --------------------------------------------------------------------------- */

int main(void)
{
    u16 n = 0;
    u8  i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    USART1_Init(19200);
    TUBES_Driver_Init();
    TUBES_SetBrightness(6);
    delay_ms(200);

    GreyTest_Selftest();

    /* 红灯 = 有模块没通过自检（以串口为准，LED 极性可能相反） */
    {
        u8 fail = 0;
        for (i = 0; i < 4; i++) if (!PING_OK[i]) fail++;
        LED_RED = fail ? 1 : 0;
    }

    for (;;) {
        GreyTest_Cycle(n);

        LED_GREEN = !LED_GREEN;                 /* 心跳 */

        if (n && (n % STATS_EVERY) == 0)  GreyTest_Stats();
#if GREY_TEST_ANALOG
        if (n && (n % ANALOG_EVERY) == 0) GreyTest_Analog();
#endif
        n++;
        delay_ms(CYCLE_MS);
    }
}
