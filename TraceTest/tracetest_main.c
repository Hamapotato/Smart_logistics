/* ============================================================================
 *  TraceTest —— 四方向循迹测试（一次只用一块灰阶 · 转向纠偏）
 *
 *  【这一版做什么】
 *      在 Keil 里改一个宏选择方向，程序就只初始化并读取那一块灰阶，
 *      然后朝那个方向走，用这块传感器把车拧回线上：
 *
 *          前进 FRONT -> 前灰阶 PB13/PB14
 *          后退 BACK  -> 后灰阶 PD1 /PD2
 *          左移 LEFT  -> 左灰阶 PC8 /PC9
 *          右移 RIGHT -> 右灰阶 PC3 /PC2
 *
 *      线偏到探头 1 那一头 -> 往那一头转，偏到另一边 -> 往另一边转。
 *      「转向」同时修两件事：车心回到线上 + 车头顺线，所以一块传感器就够。
 *
 *      - 走到十字 / 丁字路口（探头 4、5 黑，且 1~4 或 5~8 全黑）时不纠偏，
 *        直着开过去。
 *      - 丢线后按最后一次的纠偏方向继续走 TRACE_LOST_MAX 个周期（默认 2 秒），
 *        还找不回来才停住。
 *      - 起步前就一直没看到线的话，四个轮子是停着的（红灯常亮），
 *        不会上电就闷头走；选中的那块灰阶一看到线就自动起步。
 *
 *  【Keil 里只改这一个宏】
 *      #define TRACE_DIR   TRACE_DIR_FRONT
 *      可选：TRACE_DIR_FRONT / TRACE_DIR_BACK / TRACE_DIR_LEFT / TRACE_DIR_RIGHT
 *
 *  【硬件】
 *      前灰阶  软 I2C  PB13 = SCL, PB14 = SDA, 地址 0x4F
 *      后灰阶  软 I2C  PD1  = SCL, PD2  = SDA, 地址 0x4F
 *      左灰阶  软 I2C  PC8  = SCL, PC9  = SDA, 地址 0x4F
 *      右灰阶  软 I2C  PC3  = SCL, PC2  = SDA, 地址 0x4F
 *      四轮板  软 I2C  PF6  = SCL, PF5  = SDA, 地址 0x34
 *      串口    USART1  PA9  = TX, 19200 8N1
 *      绿灯 PA7 / 红灯 PA6（低电平点亮）
 *
 *  【上车前要知道的两件事】
 *      1. 每个方向有独立的符号宏 TRACE_SIGN_FRONT/BACK/LEFT/RIGHT。
 *         前灰阶已实测为 -1；另外三个方向第一次上车要单独确认：
 *         斜着往外走、越走越偏，就把该方向的符号翻成 +1。
 *      2. TRACE_BLACK_LEVEL：灰阶数字量里"黑"对应 0 还是 1。
 *         它只用于路口判据，不影响正常纠偏。设反了会在路口乱拐。
 *
 *  【轮速组合】（MoveTest 已实测确认）
 *      201 = 右前, 202 = 右后, 203 = 左后, 204 = 左前
 *      前进 (+1,+1,+1,+1)   后退 (-1,-1,-1,-1)
 *      左移 (+1,-1,+1,-1)   右移 (-1,+1,-1,+1)
 *      原地旋转 (+1,+1,-1,-1)
 *      本程序每帧 = 该方向的基础速度 + 旋转纠偏：
 *          m201 = b201 + w    m202 = b202 + w
 *          m203 = b203 - w    m204 = b204 - w
 * ========================================================================== */


#include "main.h"

/* ---------------------------------------------------------------------------
 *  一、可调参数
 * ------------------------------------------------------------------------- */

/* 直走速度。太小克服不了静摩擦，车会原地不动；Align 那边嫌 12 太慢，后来用 20。
   第一次上车先用 15，确认能走直再往上加。 */
#define TRACE_CRUISE        15

/* 纠偏增益：纠偏量 = E × 这个数 ÷ 8。8 = "纠偏量等于 E"。
   反应慢就加到 12，走起来画龙就减到 6。 */
#define TRACE_K             8

/* 纠偏限幅，**必须小于 TRACE_CRUISE**。
   否则纠偏一饱和，一侧两个轮子会被算成 "直走速度 - 纠偏量" ≤ 0，
   车就只剩另一侧两个轮子在转。下面有 #error 卡着这一条。 */
#define TRACE_MAX           5

/* 死区：|E| 小于等于它就当 0。探头是开关量，靠中心每过一个探头 E 才变约 3，
   死区太小会在量化噪声上来回抖。 */
#define TRACE_DEADBAND      2

/* ---------------------------------------------------------------------------
 *  循迹方向选择：**Keil 里只改这一行**
 *
 *      TRACE_DIR_FRONT   前进循迹，用前灰阶（已验证）
 *      TRACE_DIR_BACK    后退循迹，用后灰阶
 *      TRACE_DIR_LEFT    左移循迹，用左灰阶
 *      TRACE_DIR_RIGHT   右移循迹，用右灰阶
 *
 *  程序只会初始化、读取选中的那一块，另外三块不接线也不影响运行。
 * ------------------------------------------------------------------------- */
#define TRACE_DIR_FRONT     0
#define TRACE_DIR_BACK      1
#define TRACE_DIR_LEFT      2
#define TRACE_DIR_RIGHT     3

#define TRACE_DIR           TRACE_DIR_FRONT

/* 1 = front uses Yahboom YB-MUX04-2.0 I2C; 0 = old GW module. */
#define TRACE_FRONT_USE_YB 1

/* YB-MUX04 X1..X8 are opposite to the weight table, so reverse the byte. */
#define TRACE_YB_REVERSE_RAW 1

/* YB-MUX04 command/register that returns one digital-state byte. */
#define TRACE_YB_STATE_CMD 0x01


/* Start as soon as the selected sensor returns a valid line. */
#define TRACE_START_STABLE 1

/* Drive wheels enabled for normal line-following operation. */
#define TRACE_MOTOR_ENABLE 1

/* 纠偏方向。四块灰阶的安装方向不同，E 的符号可能相反，所以每个方向一个宏，
   互不影响。前灰阶是 2026-10-20 实车确认过的 -1；另外三个第一次上车时标定：
   车斜着往外走、越走越偏，就把对应方向的宏翻成 +1。 */
#define TRACE_SIGN_FRONT    (-1)
#define TRACE_SIGN_BACK     (-1)
#define TRACE_SIGN_LEFT     (-1)
#define TRACE_SIGN_RIGHT    (-1)

#if   (TRACE_DIR == TRACE_DIR_FRONT)
  #define TRACE_SIGN_YAW    TRACE_SIGN_FRONT
  #define TRACE_DIR_NAME    "FRONT"
#elif (TRACE_DIR == TRACE_DIR_BACK)
  #define TRACE_SIGN_YAW    TRACE_SIGN_BACK
  #define TRACE_DIR_NAME    "BACK"
#elif (TRACE_DIR == TRACE_DIR_LEFT)
  #define TRACE_SIGN_YAW    TRACE_SIGN_LEFT
  #define TRACE_DIR_NAME    "LEFT"
#elif (TRACE_DIR == TRACE_DIR_RIGHT)
  #define TRACE_SIGN_YAW    TRACE_SIGN_RIGHT
  #define TRACE_DIR_NAME    "RIGHT"
#else
  #error "TRACE_DIR must be one of TRACE_DIR_FRONT/BACK/LEFT/RIGHT"
#endif

/* 控制周期，ms。 */
#define TRACE_TICK_MS       20

/* 丢线后按最后的纠偏方向继续走多少个周期（100 × 20ms = 2 秒）。 */
#define TRACE_LOST_MAX      100

/* 每多少个周期打一行串口（25 × 20ms = 500ms）。打印是阻塞的，别打太勤。 */
#define TRACE_PRINT_EVERY   25

/* Refresh the four-digit display every 5 control frames (100 ms). */
#define TRACE_TUBE_EVERY    5

/* 灰阶数字量里"黑"对应的 bit 值（0 或 1）。只给路口判据用，见文件头。 */
#define TRACE_BLACK_LEVEL_GW       0
#define TRACE_BLACK_LEVEL_ACTIVE_HIGH 1

/* 1 = 串口输出过程；0 = 串口完全不出声（LED 照常）。 */
#define TRACE_DEBUG         1

/* 一次下发最多试几次（失败先做总线恢复再重试）。 */
#define TRACE_RETRY         3

#if (TRACE_MAX >= TRACE_CRUISE)
#error "TRACE_MAX must be smaller than TRACE_CRUISE, or one side's wheels go <= 0"
#endif

#if (TRACE_LOST_MAX < 1)
#error "TRACE_LOST_MAX must be >= 1"
#endif

/* LED 低电平点亮。 */
#define TRACE_LED_ON        0
#define TRACE_LED_OFF       1

/* 指示灯状态 */
#define TRACE_LAMP_LOCK     0   /* 绿灯常亮：线在死区里 */
#define TRACE_LAMP_CORRECT  1   /* 绿灯慢闪：正在纠偏 */
#define TRACE_LAMP_CROSS    2   /* 绿灯快闪：压在横向线上，直着过 */
#define TRACE_LAMP_COAST    3   /* 红灯快闪：丢线了，还在按最后的转向走 */
#define TRACE_LAMP_STOP     4   /* 红灯常亮：丢线超时，已停车 */
#define TRACE_LAMP_ERR      5   /* 双灯交替急闪：I2C 写驱动板失败 */

/* ---------------------------------------------------------------------------
 *  二、驱动里没有在头文件里声明的底层符号
 * ------------------------------------------------------------------------- */

/* motor.c 里 MOTOR_Data[] 是全局的，但 motor.h 没声明它。
   这里自己声明一份，就能直接调 I2C_Write_Len() 并**拿到返回值** ——
   MOTOR_CONTROL_DATA_Predit() 把返回值丢掉了，写失败在原来那条路上是看不见的。
   下标顺序必须和 MOTOR_CONTROL_DATA_Predit(&Motor_203,&Motor_202,&Motor_204,&Motor_201)
   内部一致：MOTOR_Data[0]=Motor_203, [1]=Motor_202, [2]=Motor_204, [3]=Motor_201。 */
extern int8_t MOTOR_Data[];

/* 总线恢复要用它们切 SDA 方向。 */
extern void I2C_SDA_OUT(void);
extern void I2C_SDA_IN(void);

/* 选中那块灰阶的软 I2C 底层。GREY_driver.h 只暴露了 x_GW_digital_Read()，
   而它把 read_byte 的返回值丢掉了：I2C 一失败就返回**没初始化的局部变量**，
   那是个看着合法的垃圾值，拿它开电机会出事。这里直接用带返回值的版本。 */
#if   (TRACE_DIR == TRACE_DIR_FRONT)
#if   TRACE_FRONT_USE_YB
extern sw_i2c_interface_t f_i2c_interface;
extern void f_sw_i2c_init(void);
extern int8_t f_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
  #define TRACE_SENSOR_INIT() f_sw_i2c_init()
  #define TRACE_SENSOR_NAME   "Yahboom YB-MUX04-2.0 front PB13/PB14"
  #define TRACE_SENSOR_ADDR_TEXT "0x12"
  #define TRACE_BLACK_LEVEL   TRACE_BLACK_LEVEL_GW
#else
extern sw_i2c_interface_t f_i2c_interface;
extern int8_t f_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);
  #define TRACE_I2C_IF        f_i2c_interface
  #define TRACE_I2C_READ      f_sw_i2c_read_byte
  #define TRACE_SENSOR_INIT() F_GW_digital_Init()
  #define TRACE_SENSOR_NAME   "front PB13/PB14"
  #define TRACE_SENSOR_ADDR_TEXT "0x4F"
  #define TRACE_BLACK_LEVEL   TRACE_BLACK_LEVEL_GW
#endif
#elif (TRACE_DIR == TRACE_DIR_BACK)
extern sw_i2c_interface_t b_i2c_interface;
extern int8_t b_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);
  #define TRACE_I2C_IF        b_i2c_interface
  #define TRACE_I2C_READ      b_sw_i2c_read_byte
  #define TRACE_SENSOR_INIT() B_GW_digital_Init()
  #define TRACE_SENSOR_NAME   "back PD1/PD2"
  #define TRACE_SENSOR_ADDR_TEXT "0x4F"
  #define TRACE_BLACK_LEVEL   TRACE_BLACK_LEVEL_GW
#elif (TRACE_DIR == TRACE_DIR_LEFT)
extern sw_i2c_interface_t l_i2c_interface;
extern int8_t l_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);
  #define TRACE_I2C_IF        l_i2c_interface
  #define TRACE_I2C_READ      l_sw_i2c_read_byte
  #define TRACE_SENSOR_INIT() L_GW_digital_Init()
  #define TRACE_SENSOR_NAME   "left PC8/PC9"
  #define TRACE_SENSOR_ADDR_TEXT "0x4F"
  #define TRACE_BLACK_LEVEL   TRACE_BLACK_LEVEL_GW
#else /* TRACE_DIR_RIGHT */
extern sw_i2c_interface_t r_i2c_interface;
extern int8_t r_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);
  #define TRACE_I2C_IF        r_i2c_interface
  #define TRACE_I2C_READ      r_sw_i2c_read_byte
  #define TRACE_SENSOR_INIT() R_GW_digital_Init()
  #define TRACE_SENSOR_NAME   "right PC3/PC2"
  #define TRACE_SENSOR_ADDR_TEXT "0x4F"
  #define TRACE_BLACK_LEVEL   TRACE_BLACK_LEVEL_GW
#endif

/* 0x4F 是 7 位地址，软 I2C 收发用的是 8 位形式，要左移一位。 */
#define TRACE_GW_ADDR8      ((uint8_t)(GW_GRAY_ADDR_DEF << 1))

/* YB-MUX04-2.0 responds at 7-bit I2C address 0x12. */
#define TRACE_YB_ADDR_DEF 0x12
#define TRACE_YB_ADDR8    ((uint8_t)(TRACE_YB_ADDR_DEF << 1))

/* ---------------------------------------------------------------------------
 *  三、为了过链接必须留着的 reverse()
 *
 *  工程文件清单里带着四块灰阶驱动（B/F/L/R_GREY_driver.c），它们末尾的
 *  x_GW_digital_eight() 都调用 reverse()，而它的定义在 APP\trace\trace.c ——
 *  那个文件不在本工程里。少了它就报：
 *      L6218E: Undefined symbol reverse (referred from b_grey_driver.o)
 *  本程序自己不用这个函数，纯粹为了过链接。Test_Grey / Align / MoveTest
 *  三个工程都是同样的绕法。**别当死代码删掉。**
 * ------------------------------------------------------------------------- */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}

/* ---------------------------------------------------------------------------
 *  四、灰阶：读一块、判有效性、算带符号位置
 * ------------------------------------------------------------------------- */

/* 原始字节里有多少个 1。 */
static uint8_t Trace_Popcount(uint8_t x)
{
    uint8_t n = 0;

    while (x) {
        n = (uint8_t)(n + (x & 0x01u));
        x = (uint8_t)(x >> 1);
    }
    return n;
}

/* 相邻两位不同的次数。一整段连续的黑线只会产生 1~2 次变化。 */
static uint8_t Trace_Trans(uint8_t x)
{
    uint8_t t = 0;
    uint8_t i;

    for (i = 1; i < 8; i++) {
        if (((x >> i) & 0x01u) != ((x >> (i - 1)) & 0x01u)) t++;
    }
    return t;
}

/* 这块灰阶是不是"有一条线压在探头上"。与模块极性无关：
      1 的个数 = 0 或 8  -> 整块一个颜色（没有线，或者是读回来的垃圾）
      相邻两位变化 > 2   -> 不是一整段连续的黑，是噪声
   其余都算有线。注意 1 的个数 = 4 不能排除：黑线偏宽、居中时压住 4 个探头
   正是最准的状态。 */
static uint8_t Trace_Valid(uint8_t raw)
{
    uint8_t pc = Trace_Popcount(raw);

    if ((pc == 0) || (pc == 8)) return 0;
    if (Trace_Trans(raw) > 2) return 0;

    return 1;
}

/* 黑线落在哪一头、离探头中间多远，带符号。
   bit0..bit7 = 探头 1..8，权重 +6,+4,+2,+1,-1,-2,-4,-6。
   这跟 GREY_driver.h 里 F_GW_digital_eight() 是**逐位相同**的结果：
   那个函数内部先 reverse(~raw) 再按同一组权重求和，因为权重反对称
   （w[i] = -w[7-i]、总和为 0），reverse 和取反恰好抵消。
   直接对原始字节加权还能省掉一次 I2C 读。

   返回值范围 ±13（最大子集和 = 6+4+2+1），靠中心每过一个探头约变 3 个单位。
   0 = 线在正中间；正数表示线偏向探头 1 那一头。换模块极性只是整体变号，
   会被 TRACE_SIGN_YAW 吸收，所以程序不需要知道到底是哪种极性。 */
static int16_t Trace_Pos(uint8_t raw)
{
    static const int8_t W[8] = { 6, 4, 2, 1, -1, -2, -4, -6 };
    int16_t s = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (raw & (uint8_t)(1u << i)) s = (int16_t)(s + W[i]);
    }
    return s;
}

/* 是不是"压在横向线（十字 / 丁字路口）上了"。
   跟线时黑线一般只压在 1~3 个探头上；走到路口时横向那条线
   和探头排平行，会同时压住纵向线的中间两个和横向一整条。判据看**形状**：
      探头 4、5 是黑，且（1~4 全黑 或 5~8 全黑）
   注意它不依赖 Trace_Valid —— 压住一整侧的时候交界只有 1~2 处，
   Trace_Valid 照样判"有线"。
   注意：这条判据依赖知道"黑"是 0 还是 1（TRACE_BLACK_LEVEL），见文件头。 */
static uint8_t Trace_Blob(uint8_t raw)
{
    uint8_t b  = (uint8_t)TRACE_BLACK_LEVEL;
    uint8_t lo = (uint8_t)(b ? 0x0Fu : 0x00u);
    uint8_t hi = (uint8_t)(b ? 0xF0u : 0x00u);

    /* 探头 4 = bit3，探头 5 = bit4 */
    if ((uint8_t)((raw >> 3) & 1u) != b) return 0;
    if ((uint8_t)((raw >> 4) & 1u) != b) return 0;

    if ((uint8_t)(raw & 0x0Fu) == lo) return 1;     /* 左侧 1~4 号全黑 */
    if ((uint8_t)(raw & 0xF0u) == hi) return 1;     /* 右侧 5~8 号全黑 */

    return 0;
}

/* YB-MUX04 channel bits are shifted by one position. */
static uint8_t Trace_YB_MapChannels(uint8_t raw)
{
    return (uint8_t)((raw >> 1) | (raw << 7));
}

/* 读一次选中的灰阶。返回 1 = I2C 成功，0 = 失败（*raw 不改）。 */
static uint8_t Trace_ReadRaw(uint8_t *raw)
{
    uint8_t v = 0xFF;

#if   (TRACE_DIR == TRACE_DIR_FRONT) && TRACE_FRONT_USE_YB
    if (f_sw_i2c_mem_read(&f_i2c_interface, TRACE_YB_ADDR8,
                          TRACE_YB_STATE_CMD, &v, 1) != 0) {
        *raw = 0xFF;
        return 0;
    }
#if   TRACE_YB_REVERSE_RAW
    v = reverse(v);
#endif
    v = Trace_YB_MapChannels(v);
#else
    if (TRACE_I2C_READ(&TRACE_I2C_IF, TRACE_GW_ADDR8, &v) != 0) return 0;
#endif

    *raw = v;
    return 1;
}

/* raw bit0..bit7 = physical leftmost ch1..rightmost ch8.
   Show up to four black-channel numbers, right aligned. Clear if none/error/more than four. */
static void Trace_ShowBlackChannels(uint8_t raw, uint8_t read_ok)
{
    unsigned char digits[4] = {10, 10, 10, 10};
    uint8_t ch;
    uint8_t count = 0;
    uint8_t pos;

    if (read_ok) {
        for (ch = 0; ch < 8; ch++) {
            if ((uint8_t)((raw >> ch) & 1u) == TRACE_BLACK_LEVEL) count++;
        }
    }

    if ((count > 0) && (count <= 4)) {
        pos = (uint8_t)(4 - count);
        for (ch = 0; ch < 8; ch++) {
            if ((uint8_t)((raw >> ch) & 1u) == TRACE_BLACK_LEVEL) {
                digits[pos++] = (unsigned char)(ch + 1);
            }
        }
    }

    TUBES_DisplayDigits(digits);
}

/* ---------------------------------------------------------------------------
 *  五、纠偏律
 * ------------------------------------------------------------------------- */

/* E -> 纠偏量 w。死区 -> 限幅 -> 符号，顺序和 Align 的阶段 B 一致。
   E 是整数权重，所以先乘后除（K=8 时正好等于 E 本身）。 */
static int16_t Trace_Steer(int16_t e)
{
    int16_t v;

    if ((e >= -TRACE_DEADBAND) && (e <= TRACE_DEADBAND)) return 0;

    v = (int16_t)((e * TRACE_K) / 8);

    if (v > TRACE_MAX)  v = TRACE_MAX;
    if (v < -TRACE_MAX) v = -TRACE_MAX;

    return (int16_t)(TRACE_SIGN_YAW * v);
}

/* ---------------------------------------------------------------------------
 *  六、电机下发
 * ------------------------------------------------------------------------- */

/* 总线恢复：从机把 SDA 拉死不放时，主机在 SCL 上补 9 个脉冲通常能让它
   把剩下的位吐完、自己松手；最后再补一个 STOP。不这样做的话，一旦总线卡住，
   后面每一次下发都是失败的，而驱动板会一直保持上一次的转速。 */
static void Trace_I2C_Recover(void)
{
    uint8_t i;

    I2C_SDA_IN();               /* 先放开 SDA，别跟从机对着拉 */
    for (i = 0; i < 9; i++) {
        IIC_SCL = 0;
        delay_us(5);
        IIC_SCL = 1;
        delay_us(5);
    }
    I2C_SDA_OUT();              /* 补一个 STOP：SCL 高时 SDA 由低变高 */
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 1;
    delay_us(5);
    IIC_SDA = 1;
    delay_us(5);
}

/* 按选中的方向把基础速度混成四轮。
   基础速度：FRONT (+,+,+,+)   BACK  (-,-,-,-)
             LEFT  (+,-,+,-)   RIGHT (-,+,-,+)
   再加原地旋转 (+1,+1,-1,-1) * w。
   注意：w 的符号仍由各方向的 TRACE_SIGN_xxx 决定。 */
static void Trace_Mix(int16_t vx, int16_t w,
                      int16_t *m201, int16_t *m202,
                      int16_t *m203, int16_t *m204)
{
    int16_t b201, b202, b203, b204;

#if   (TRACE_DIR == TRACE_DIR_FRONT)
    b201 = vx;              b202 = vx;
    b203 = vx;              b204 = vx;
#elif (TRACE_DIR == TRACE_DIR_BACK)
    b201 = (int16_t)(-vx);  b202 = (int16_t)(-vx);
    b203 = (int16_t)(-vx);  b204 = (int16_t)(-vx);
#elif (TRACE_DIR == TRACE_DIR_LEFT)
    b201 = vx;              b202 = (int16_t)(-vx);
    b203 = vx;              b204 = (int16_t)(-vx);
#else /* TRACE_DIR_RIGHT */
    b201 = (int16_t)(-vx);  b202 = vx;
    b203 = (int16_t)(-vx);  b204 = vx;
#endif

    *m201 = (int16_t)(b201 + w);
    *m202 = (int16_t)(b202 + w);
    *m203 = (int16_t)(b203 - w);
    *m204 = (int16_t)(b204 - w);
}

/* 按"该方向基础速度 + 原地旋转"下发。
   返回 0 = 写成功，1 = 几次重试都失败。 */
static uint8_t Trace_Send(int16_t m201, int16_t m202, int16_t m203, int16_t m204)
{
    uint8_t r;

    MOTOR_Data[0] = (int8_t)m203;
    MOTOR_Data[1] = (int8_t)m202;
    MOTOR_Data[2] = (int8_t)m204;
    MOTOR_Data[3] = (int8_t)m201;

    for (r = 0; r < TRACE_RETRY; r++) {
        if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, MOTOR_COUNT) == 0)
            return 0;
        Trace_I2C_Recover();
    }
    return 1;
}

/* ---------------------------------------------------------------------------
 *  七、指示灯
 * ------------------------------------------------------------------------- */

/* tick 的单位是 TRACE_TICK_MS（20ms）。 */
static void Trace_Lamp(uint8_t lamp, uint16_t tick)
{
    switch (lamp) {
        case TRACE_LAMP_LOCK:                   /* 常亮 */
            LED_GREEN = TRACE_LED_ON;
            LED_RED   = TRACE_LED_OFF;
            break;
        case TRACE_LAMP_CORRECT:                /* 200ms 翻转 = 慢闪 */
            LED_GREEN = ((tick / 10) & 1) ? TRACE_LED_OFF : TRACE_LED_ON;
            LED_RED   = TRACE_LED_OFF;
            break;
        case TRACE_LAMP_CROSS:                  /* 60ms 翻转 = 快闪 */
            LED_GREEN = ((tick / 3) & 1) ? TRACE_LED_OFF : TRACE_LED_ON;
            LED_RED   = TRACE_LED_OFF;
            break;
        case TRACE_LAMP_COAST:                  /* 红灯快闪 */
            LED_RED   = ((tick / 3) & 1) ? TRACE_LED_OFF : TRACE_LED_ON;
            LED_GREEN = TRACE_LED_OFF;
            break;
        case TRACE_LAMP_STOP:                   /* 红灯常亮 */
            LED_RED   = TRACE_LED_ON;
            LED_GREEN = TRACE_LED_OFF;
            break;
        case TRACE_LAMP_ERR:                    /* 双灯交替急闪 */
            LED_RED   = ((tick / 2) & 1) ? TRACE_LED_ON : TRACE_LED_OFF;
            LED_GREEN = ((tick / 2) & 1) ? TRACE_LED_OFF : TRACE_LED_ON;
            break;
        default:
            LED_RED   = TRACE_LED_OFF;
            LED_GREEN = TRACE_LED_OFF;
            break;
    }
}

/* ---------------------------------------------------------------------------
 *  八、主循环
 * ------------------------------------------------------------------------- */

int main(void)
{
    uint8_t  i;
    uint8_t  raw = 0;
    uint8_t  ok;
    uint8_t  blob;
    uint8_t  lamp;
    uint8_t  started = 0;
    uint8_t  io_err_ticks = 0;
    uint16_t tick = 0;
    uint16_t lost = 0;
    int16_t  e = 0;
    int16_t  w = 0;
    int16_t  w_hold = 0;
    int16_t  vx;
    int16_t  m201, m202, m203, m204;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);              /* delay_us / delay_ms 要用，必须最先做 */
    LED_Init();
    USART1_Init(19200);
    TUBES_Driver_Init();
    TUBES_SetBrightness(6);
    TUBES_Clear();

    LED_RED   = TRACE_LED_OFF;
    LED_GREEN = TRACE_LED_OFF;

    MOTOR_Init();
    delay_ms(5);

    /* 上电第一件事：先让四个轮子停下，并且检查这次写有没有成功。 */
    for (i = 0; i < 3; i++) {
        if (Trace_Send(0, 0, 0, 0) == 0) break;
        delay_ms(50);
    }

    TRACE_SENSOR_INIT();
    delay_ms(300);

#if TRACE_DEBUG
    printf("\r\n== TraceTest (%s line following) ==\r\n", TRACE_DIR_NAME);
    printf("TraceTest: dir=%s sensor=%s addr=%s black=%d\r\n",
           TRACE_DIR_NAME, TRACE_SENSOR_NAME, TRACE_SENSOR_ADDR_TEXT,
           (int)TRACE_BLACK_LEVEL);
    printf("TraceTest: cruise=%d  k=%d/8  max=%d  deadband=%d  sign_yaw=%d\r\n",
           (int)TRACE_CRUISE, (int)TRACE_K, (int)TRACE_MAX,
           (int)TRACE_DEADBAND, (int)TRACE_SIGN_YAW);
    printf("TraceTest: motor board PF6=SCL PF5=SDA @0x34\r\n");
    printf("TraceTest: green SOLID=on line  green SLOW=correcting  green FAST=crossing\r\n");
    printf("TraceTest: red FAST=lost, still moving  red SOLID=lost, stopped  BOTH=write failed\r\n");
#endif

    for (;;) {
        /* ---------- 1. 读选中的灰阶 ---------- */
        ok = Trace_ReadRaw(&raw);
        if ((tick % TRACE_TUBE_EVERY) == 0) {
            Trace_ShowBlackChannels(raw, ok);
        }
        if (ok) ok = Trace_Valid(raw);

        /* ---------- 2. 算纠偏 ---------- */
        if (!started && ok) started = 1;

        if (!started) {
            /* 还没见过线：停在原地等，别上电就闷头往前开 */
            e    = 0;
            w    = 0;
            vx   = 0;
            lamp = TRACE_LAMP_STOP;
        } else if (ok) {
            lost = 0;
            blob = Trace_Blob(raw);
            e    = Trace_Pos(raw);

            if (blob) {
                /* 压在横向线上：这一帧的 E 是被整块黑拉偏的，不能拿它转向 */
                w      = 0;
                w_hold = 0;
                lamp   = TRACE_LAMP_CROSS;
            } else {
                w      = Trace_Steer(e);
                w_hold = w;             /* 丢线时照着这一帧的转向接着拧 */
                lamp   = (w == 0) ? TRACE_LAMP_LOCK : TRACE_LAMP_CORRECT;
            }
            vx = TRACE_CRUISE;
        } else {
            e = 0;
            if (lost < 0xFFFF) lost++;

            if (lost <= TRACE_LOST_MAX) {
                w    = w_hold;          /* 按最后一次有效纠偏继续走 */
                lamp = TRACE_LAMP_COAST;
                vx   = TRACE_CRUISE;
            } else {
                w    = 0;
                lamp = TRACE_LAMP_STOP;
                vx   = 0;
            }
        }

#if !TRACE_MOTOR_ENABLE
        vx = 0;
        w = 0;
        lamp = TRACE_LAMP_STOP;
#endif

        /* ---------- 3. 混成四个轮速 ---------- */
        Trace_Mix(vx, w, &m201, &m202, &m203, &m204);

        /* ---------- 4. 下发给四个轮子 ---------- */
        if (Trace_Send(m201, m202, m203, m204) != 0) {
            io_err_ticks = 40;
        }
        if (io_err_ticks > 0) {
            io_err_ticks--;
            lamp = TRACE_LAMP_ERR;
        }

        /* ---------- 5. 指示灯 + 串口 ---------- */
        Trace_Lamp(lamp, tick);

#if TRACE_DEBUG
        if ((tick % TRACE_PRINT_EVERY) == 0) {
            printf("Trace[%s]: raw %02X ok %d E %3d w %2d | m201 %3d m202 %3d m203 %3d m204 %3d | %s\r\n",
                   TRACE_DIR_NAME,
                   (unsigned int)raw, (int)ok, (int)e, (int)w,
                   (int)m201, (int)m202, (int)m203, (int)m204,
                   (lamp == TRACE_LAMP_LOCK)    ? "LOCK"  :
                   (lamp == TRACE_LAMP_CORRECT) ? "CORR"  :
                   (lamp == TRACE_LAMP_CROSS)   ? "CROSS" :
                   (lamp == TRACE_LAMP_COAST)   ? "LOST"  :
                   (lamp == TRACE_LAMP_STOP)    ? "STOP"  : "ERR");
        }
#endif

        tick++;
        delay_ms(TRACE_TICK_MS);
    }
}
