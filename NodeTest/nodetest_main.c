/* ============================================================================
 * NodeTest v8 —— 固定节点任务序列、四方向循迹与节点停车
 *
 * 【固定任务序列】
 *   前进 -> 停车 -> 右移 -> 停车 -> 前进 -> 停车 -> 右移 -> 停车
 *   -> 前进 -> 停车 -> 左移 -> 停车 -> 后退 -> 停车
 *
 *   每一步使用移动方向及其反向的两块八路灰阶循迹，
 *   另外两块负责检测下一节点。到达节点后先反向制动，再停车。
 *
 * 【底盘驱动】
 *   参考 MoveTest，直接向幻尔四路驱动板写四轮目标速度。
 *   不使用编码器速度闭环，不使用 0x1F PWM。
 *
 * 【灰阶】
 *   前：PB13=SCL，PB14=SDA
 *   后：PD1 =SCL，PD2 =SDA
 *   左：PC8 =SCL，PC9 =SDA
 *   右：PC3 =SCL，PC2 =SDA
 *   四块均为亚博 YB-MUX04-2.0，地址 0x12，命令 0x01。
 * ========================================================================== */

#include "main.h"

/* ---------------------------------------------------------------------------
 *  一、可调参数
 * ------------------------------------------------------------------------- */

/* 节点间直行速度，单位与 MoveTest 的 0x33 一致。 */
#define NODE_CRUISE                 10

/* 前灰阶循迹纠偏。 */
#define NODE_STEER_K                6
#define NODE_STEER_MAX              2
#define NODE_STEER_SIGN             (-1)
#define NODE_STEER_DEADBAND         2

/* 2026-10-27 FourDirStraightTest 实车验证参数：
   平移 K=8、限幅 2；旋转 K=8、限幅 1；死区 2。
   平移符号四个方向均为 +1。
   旋转符号前进/后退为 +1，左移/右移为 -1。 */
#define NODE_TRACK_LAT_K            8
#define NODE_TRACK_LAT_MAX          2
#define NODE_TRACK_LAT_SIGN         (+1)
#define NODE_TRACK_LONG_MAX         2
#define NODE_TRACK_LONG_SIGN        (+1)
#define NODE_TRACK_YAW_K            8
#define NODE_TRACK_YAW_MAX          1
#define NODE_TRACK_YAW_SIGN         (+1)  /* forward */
#define NODE_BACK_YAW_SIGN          (-1)  /* backward */
#define NODE_SIDE_YAW_SIGN          (-1)  /* left/right */
#define NODE_TRACK_DEADBAND         2

/* 节点居中修正速度与每步时间。速度低一些，避免冲过节点。 */
#define NODE_CORRECT_SPEED          2
#define NODE_CORRECT_STEP_MS        40
#define NODE_CORRECT_DEADBAND       2
#define NODE_CORRECT_MAX_STEPS      160

/* 控制周期。 */
#define NODE_TICK_MS                20

/* 上电后先等一小段时间，再检查节点中心。 */
#define NODE_START_DELAY_MS         1000

/* 节点中心判断连续成立次数。 */
#define NODE_CONFIRM_COUNT          2

/* 右灰阶的左侧两路为黑就提前停车，随后再低速居中。
   左侧两路是探头 1、2，对应 mask 0x03。 */
#define NODE_STOP_LOW_MASK          0x03
#define NODE_STOP_HIGH_MASK         0xC0
#define NODE_ARM_DELAY_TICKS        25

/* 0 = skip post-stop centering, 1 = keep Node_Center() enabled. */
#define NODE_POST_STOP_CENTER        0
#define NODE_BRAKE_SPEED            1
#define NODE_BRAKE_MS               400

#define NODE_MODE_FORWARD            0
#define NODE_MODE_BACK               1
#define NODE_MODE_LEFT               2
#define NODE_MODE_RIGHT              3

/* 固定节点任务。每一步都会行驶到下一节点并停车。 */
#define NODE_TASK_COUNT              7U
static const uint8_t NODE_TASK_MODES[NODE_TASK_COUNT] =
{
    NODE_MODE_FORWARD,
    NODE_MODE_RIGHT,
    NODE_MODE_FORWARD,
    NODE_MODE_RIGHT,
    NODE_MODE_FORWARD,
    NODE_MODE_LEFT,
    NODE_MODE_BACK
};

#define NODE_IR_CONFIRM              2
#define NODE_IR_POLL_MS              20
#define NODE_IR_ACTIVE_LOW           1

#if NODE_IR_ACTIVE_LOW
  #define NODE_IR_READ(port, pin) (!GPIO_ReadInputDataBit((port), (pin)))
#else
  #define NODE_IR_READ(port, pin) (GPIO_ReadInputDataBit((port), (pin)) != 0)
#endif

#define NODE_IR_FRONT_L NODE_IR_READ(GPIOD, GPIO_Pin_13)
#define NODE_IR_FRONT_R NODE_IR_READ(GPIOE, GPIO_Pin_12)

/* 停车后等待惯性衰减，再进入居中修正。 */
#define NODE_INERTIA_SETTLE_MS      150

/* 节点间行驶超时。 */
#define NODE_TRAVEL_TIMEOUT_TICKS   500

/* 居中修正阶段超时。 */
#define NODE_CENTER_TIMEOUT_TICKS   (NODE_CORRECT_MAX_STEPS * NODE_CORRECT_STEP_MS / NODE_TICK_MS)

/* 串口打印间隔。 */
#define NODE_PRINT_EVERY            25

/* 1 = 输出启动、节点和失败信息；0 = 关闭串口调试。 */
#define NODE_DEBUG                  1

/* I2C 写入重试次数。 */
#define NODE_RETRY                  3

/* 灰阶黑线极性：0 表示原始 bit=0 为黑。 */
#define NODE_BLACK_LEVEL            1

/* 中间两路探头 4、5。 */
#define NODE_CENTER_MASK            0x18

/* 节点居中时只允许根据八路灰阶实际读到的黑线方向修正。
 *
 * 前灰阶：
 *   右侧两路黑 -> 低速右移
 *   左侧两路黑 -> 低速左移
 *
 * 右灰阶：
 *   左侧两路黑 -> 低速前进
 *   右侧两路黑 -> 低速后退
 *
 * 如果模块安装方向相反，只交换下面两组 mask。
 */
#define NODE_FRONT_RIGHT_MASK       0x03
#define NODE_FRONT_RIGHT_HALF       0x0F
#define NODE_FRONT_LEFT_MASK        0xC0
#define NODE_FRONT_LEFT_HALF        0xF0

/* 只作用于 Node_Center() 的左右修正，不改变循迹或传感器识别。 */
#define NODE_CORRECT_LATERAL_SIGN   (-1)

#define NODE_RIGHT_FORWARD_MASK     0x03
#define NODE_RIGHT_FORWARD_HALF     0x0F
#define NODE_RIGHT_BACK_MASK        0xC0
#define NODE_RIGHT_BACK_HALF        0xF0

#define NODE_LED_ON                 0
#define NODE_LED_OFF                1

/* 右侧灰阶底层接口。 */
extern sw_i2c_interface_t f_i2c_interface;
extern sw_i2c_interface_t b_i2c_interface;
extern sw_i2c_interface_t l_i2c_interface;
extern sw_i2c_interface_t r_i2c_interface;
extern int8_t f_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t b_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t l_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t r_sw_i2c_mem_read(sw_i2c_interface_t *,
                                uint8_t, uint8_t, uint8_t *, uint8_t);
extern void f_sw_i2c_init(void);
extern void b_sw_i2c_init(void);
extern void l_sw_i2c_init(void);
extern void r_sw_i2c_init(void);

#define NODE_GW_ADDR8 ((uint8_t)(GW_GRAY_ADDR_DEF << 1))

#define NODE_YB_ADDR_DEF 0x12
#define NODE_YB_ADDR8    ((uint8_t)(NODE_YB_ADDR_DEF << 1))
#define NODE_YB_STATE_CMD 0x01

uint8_t reverse(uint8_t x);

extern int8_t MOTOR_Data[];
extern void I2C_SDA_OUT(void);
extern void I2C_SDA_IN(void);

/* ---------------------------------------------------------------------------
 *  二、底盘下发
 * ------------------------------------------------------------------------- */

static void Node_I2C_Recover(void)
{
    uint8_t i;

    I2C_SDA_IN();
    for (i = 0; i < 9; i++)
    {
        IIC_SCL = 0;
        delay_us(5);
        IIC_SCL = 1;
        delay_us(5);
    }
    I2C_SDA_OUT();
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 1;
    delay_us(5);
    IIC_SDA = 1;
    delay_us(5);
}

/* body 顺序：201=右前，202=右后，203=左后，204=左前。 */
static uint8_t Node_WriteBody(const int16_t body[4])
{
    uint8_t retry;

    MOTOR_Data[0] = (int8_t)body[2]; /* CH1 = 203 */
    MOTOR_Data[1] = (int8_t)body[1]; /* CH2 = 202 */
    MOTOR_Data[2] = (int8_t)body[3]; /* CH3 = 204 */
    MOTOR_Data[3] = (int8_t)body[0]; /* CH4 = 201 */

    for (retry = 0; retry < NODE_RETRY; retry++)
    {
        if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, 4) == 0)
            return 0;
        Node_I2C_Recover();
    }
    return 1;
}

/* vx>0 前进，vy>0 左移，w 为原地旋转。 */
static uint8_t Node_MoveVectorW(int16_t vx, int16_t vy, int16_t w)
{
    int16_t body[4];

    body[0] = (int16_t)(vx + vy); /* 201 */
    body[1] = (int16_t)(vx - vy); /* 202 */
    body[2] = (int16_t)(vx + vy); /* 203 */
    body[3] = (int16_t)(vx - vy); /* 204 */

    body[0] = (int16_t)(body[0] + w);
    body[1] = (int16_t)(body[1] + w);
    body[2] = (int16_t)(body[2] - w);
    body[3] = (int16_t)(body[3] - w);
    return Node_WriteBody(body);
}

static uint8_t Node_MoveVector(int16_t vx, int16_t vy)
{
    return Node_MoveVectorW(vx, vy, 0);
}

static void Node_Stop(void)
{
    int16_t zero[4] = { 0, 0, 0, 0 };
    Node_WriteBody(zero);
}

/* ---------------------------------------------------------------------------
 *  三、灰阶读取、节点判断、连续位置
 * ------------------------------------------------------------------------- */

static uint8_t Node_ReadFront(uint8_t *raw)
{
    return (f_sw_i2c_mem_read(&f_i2c_interface, NODE_YB_ADDR8,
                              NODE_YB_STATE_CMD, raw, 1) == 0)
         ? 1U : 0U;
}

static uint8_t Node_ReadBack(uint8_t *raw)
{
    return (b_sw_i2c_mem_read(&b_i2c_interface, NODE_YB_ADDR8,
                              NODE_YB_STATE_CMD, raw, 1) == 0)
         ? 1U : 0U;
}

static uint8_t Node_ReadRight(uint8_t *raw)
{
    return (r_sw_i2c_mem_read(&r_i2c_interface, NODE_YB_ADDR8,
                              NODE_YB_STATE_CMD, raw, 1) == 0)
         ? 1U : 0U;
}

static uint8_t Node_Popcount(uint8_t x)
{
    uint8_t n = 0;
    while (x)
    {
        n = (uint8_t)(n + (x & 1U));
        x = (uint8_t)(x >> 1);
    }
    return n;
}

static uint8_t Node_Transitions(uint8_t x)
{
    uint8_t t = 0;
    uint8_t i;
    for (i = 1; i < 8; i++)
    {
        if (((x >> i) & 1U) != ((x >> (i - 1)) & 1U)) t++;
    }
    return t;
}

static uint8_t Node_Valid(uint8_t raw)
{
    uint8_t pc = Node_Popcount(raw);
    if ((pc == 0) || (pc == 8)) return 0;
    if (Node_Transitions(raw) > 2) return 0;
    return 1;
}

/* Same conversion as StraightTest front YB-MUX04-2.0. */
static uint8_t Node_YBBits(uint8_t raw)
{
    uint8_t v = reverse(raw);
    v = (uint8_t)((v >> 1) | (v << 7));
    return (uint8_t)(~v);
}

static uint8_t Node_EdgeMask(uint8_t bits)
{
    uint8_t middle = (uint8_t)(bits & 0x18U);
    if (middle != 0x18U) return bits;
    if ((bits & 0x07U) == 0x07U) return 0;
    if ((bits & 0xE0U) == 0xE0U) return 0;
    return bits;
}

/* 返回中间两路是否为黑。 */
static uint8_t Node_MaskBlack(uint8_t raw, uint8_t mask)
{
#if NODE_BLACK_LEVEL
    return ((raw & mask) == mask) ? 1U : 0U;
#else
    return (((uint8_t)(~raw) & mask) == mask) ? 1U : 0U;
#endif
}

static uint8_t Node_BlackBits(uint8_t raw)
{
#if NODE_BLACK_LEVEL
    return raw;
#else
    return (uint8_t)(~raw);
#endif
}

/* 节点中心：前灰阶和右灰阶的探头 4、5 同时为黑。 */
static uint8_t Node_IsCenter(uint8_t front_bits, uint8_t right_raw)
{
    return (uint8_t)(((front_bits & NODE_CENTER_MASK) == NODE_CENTER_MASK)
                   && Node_MaskBlack(right_raw, NODE_CENTER_MASK));
}

/* 灰阶连续位置，权重与 TraceTest 相同。 */
static int16_t Node_LinePos(uint8_t raw)
{
    static const int8_t W[8] = { 6, 4, 2, 1, -1, -2, -4, -6 };
    int16_t pos = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (raw & (uint8_t)(1U << i)) pos = (int16_t)(pos + W[i]);
    }
    return pos;
}

static int16_t Node_Abs16(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

static int16_t Node_Weighted(uint8_t bits, const int8_t weight[8])
{
    int16_t sum = 0;
    uint8_t i;
    for (i = 0; i < 8; i++)
        if (bits & (uint8_t)(1U << i)) sum = (int16_t)(sum + weight[i]);
    return sum;
}

static int16_t Node_TrackLat(uint8_t bits, uint8_t back)
{
    static const int8_t FRONT_W[8] = { +8, +5, +2, +1, -1, -2, -5, -8 };
    static const int8_t BACK_W[8]  = { +8, +5, +2, +1, -1, -2, -5, -8 };
    return Node_Weighted(bits, back ? BACK_W : FRONT_W);
}

static int16_t Node_TrackYaw(uint8_t bits)
{
    static const int8_t YAW_W[8] = { +8, +5, +2, +1, -1, -2, -5, -8 };
    return Node_Weighted(bits, YAW_W);
}

static int16_t Node_TrackChannel(int16_t e, int16_t k,
                                 int16_t limit, int8_t sign)
{
    int16_t v = (int16_t)((e * k) / 8);
    if (v > limit) v = limit;
    if (v < -limit) v = -limit;
    return (int16_t)(sign * v);
}

/* 前灰阶转向纠偏量，控制律与 TraceTest 一致。 */
static int16_t Node_Steer(int16_t e)
{
    int16_t w;

    if ((e >= -NODE_STEER_DEADBAND) && (e <= NODE_STEER_DEADBAND))
        return 0;

    w = (int16_t)((e * NODE_STEER_K) / 8);
    if (w > NODE_STEER_MAX) w = NODE_STEER_MAX;
    if (w < -NODE_STEER_MAX) w = -NODE_STEER_MAX;
    return (int16_t)(NODE_STEER_SIGN * w);
}

/* ---------------------------------------------------------------------------
 *  四、节点中心低速修正
 * ------------------------------------------------------------------------- */

/* 读前后两个传感器；返回 0 表示本次读取失败。 */
static uint8_t Node_ReadBoth(uint8_t *front_raw, uint8_t *right_raw)
{
    if (Node_ReadFront(front_raw) == 0) return 0;
    if (Node_ReadRight(right_raw) == 0) return 0;
    return 1;
}

/*
 * 低速修正到节点中心。
 *
 * 只根据八路灰阶实际读到的线修正：
 *   - 前灰阶右侧黑：右移；前灰阶左侧黑：左移；
 *   - 右灰阶左侧黑：前进；右灰阶右侧黑：后退。
 *
 * 两块灰阶都没有有效线时只停车等待，绝不盲目前后左右搜索。
 * 每次只走一个短步，然后停车重新读取。
 */
#define NODE_READ_FRONT 0x01U
#define NODE_READ_BACK  0x02U
#define NODE_READ_LEFT  0x04U
#define NODE_READ_RIGHT 0x08U

typedef struct
{
    uint8_t front_raw;
    uint8_t back_raw;
    uint8_t left_raw;
    uint8_t right_raw;
    uint8_t front_bits;
    uint8_t back_bits;
    uint8_t left_bits;
    uint8_t right_bits;
    uint8_t front_ctrl;
    uint8_t back_ctrl;
    uint8_t left_ctrl;
    uint8_t right_ctrl;
    uint8_t front_ok;
    uint8_t back_ok;
    uint8_t left_ok;
    uint8_t right_ok;
} NodeSensors;

typedef struct
{
    int16_t lat;
    int16_t lon;
    int16_t yaw;
    uint8_t valid_count;
} NodePose;

static uint8_t Node_ReadLeft(uint8_t *raw)
{
    return (l_sw_i2c_mem_read(&l_i2c_interface, NODE_YB_ADDR8,
                              NODE_YB_STATE_CMD, raw, 1) == 0)
         ? 1U : 0U;
}

static uint8_t Node_ReadSensors(NodeSensors *s)
{
    uint8_t read_ok = 0;

    s->front_raw = s->back_raw = s->left_raw = s->right_raw = 0;
    s->front_bits = s->back_bits = s->left_bits = s->right_bits = 0;
    s->front_ctrl = s->back_ctrl = s->left_ctrl = s->right_ctrl = 0;
    s->front_ok = s->back_ok = s->left_ok = s->right_ok = 0;

    if (Node_ReadFront(&s->front_raw)) {
        read_ok |= NODE_READ_FRONT;
        s->front_bits = Node_YBBits(s->front_raw);
        s->front_ok = Node_Valid(s->front_bits);
        if (s->front_ok) s->front_ctrl = Node_EdgeMask(s->front_bits);
    }
    if (Node_ReadBack(&s->back_raw)) {
        read_ok |= NODE_READ_BACK;
        s->back_bits = Node_YBBits(s->back_raw);
        s->back_ok = Node_Valid(s->back_bits);
        if (s->back_ok) s->back_ctrl = Node_EdgeMask(s->back_bits);
    }
    if (Node_ReadLeft(&s->left_raw)) {
        read_ok |= NODE_READ_LEFT;
        /* Left module is 8..1 front-to-back; reverse to 1..8. */
        s->left_bits = reverse(Node_YBBits(s->left_raw));
        s->left_ok = Node_Valid(s->left_bits);
        if (s->left_ok) s->left_ctrl = Node_EdgeMask(s->left_bits);
    }
    if (Node_ReadRight(&s->right_raw)) {
        read_ok |= NODE_READ_RIGHT;
        /* Right module is already 1..8 front-to-back. */
        s->right_bits = Node_YBBits(s->right_raw);
        s->right_ok = Node_Valid(s->right_bits);
        if (s->right_ok) s->right_ctrl = Node_EdgeMask(s->right_bits);
    }

    return read_ok;
}

static void Node_GetPoseCenter(const NodeSensors *s, NodePose *p)
{
    p->lat = 0;
    p->lon = 0;
    p->yaw = 0;
    p->valid_count = 0;

    if (s->front_ok) {
        p->lat = (int16_t)(p->lat + Node_TrackLat(s->front_ctrl, 0));
        p->valid_count++;
    }
    if (s->back_ok) {
        p->lat = (int16_t)(p->lat + Node_TrackLat(s->back_ctrl, 1));
        p->valid_count++;
    }
    if (s->left_ok) {
        p->lon = (int16_t)(p->lon + Node_TrackLat(s->left_ctrl, 0));
        p->valid_count++;
    }
    if (s->right_ok) {
        p->lon = (int16_t)(p->lon + Node_TrackLat(s->right_ctrl, 0));
        p->valid_count++;
    }

    /* Rotation is a difference, not a sum, between opposite sensors. */
    if (s->front_ok && s->back_ok)
        p->yaw = (int16_t)(p->yaw
                + Node_TrackYaw(s->front_ctrl)
                - Node_TrackYaw(s->back_ctrl));
    if (s->left_ok && s->right_ok)
        p->yaw = (int16_t)(p->yaw
                + Node_TrackYaw(s->left_ctrl)
                - Node_TrackYaw(s->right_ctrl));
}

static void Node_GetPoseTravel(const NodeSensors *s, NodePose *p,
                                uint8_t mode)
{
    p->lat = 0;
    p->lon = 0;
    p->yaw = 0;
    p->valid_count = 0;

    if ((mode == NODE_MODE_FORWARD) || (mode == NODE_MODE_BACK))
    {
        if (s->front_ok) {
            p->lat = (int16_t)(p->lat + Node_TrackLat(s->front_ctrl, 0));
            p->valid_count++;
        }
        if (s->back_ok) {
            p->lat = (int16_t)(p->lat + Node_TrackLat(s->back_ctrl, 1));
            p->valid_count++;
        }
        if (s->front_ok && s->back_ok)
            p->yaw = (int16_t)(Node_TrackYaw(s->front_ctrl)
                             - Node_TrackYaw(s->back_ctrl));
    }
    else
    {
        if (s->left_ok) {
            p->lon = (int16_t)(p->lon + Node_TrackLat(s->left_ctrl, 0));
            p->valid_count++;
        }
        if (s->right_ok) {
            p->lon = (int16_t)(p->lon + Node_TrackLat(s->right_ctrl, 0));
            p->valid_count++;
        }
        if (s->left_ok && s->right_ok)
            p->yaw = (int16_t)(Node_TrackYaw(s->left_ctrl)
                             - Node_TrackYaw(s->right_ctrl));
    }
}

static int8_t Node_SignStep(int16_t value, int16_t deadband, int8_t sign)
{
    if (value > deadband) return (int8_t)((sign > 0) ? 1 : -1);
    if (value < -deadband) return (int8_t)((sign > 0) ? -1 : 1);
    return 0;
}

static uint8_t Node_Center(void)
{
    uint16_t step;
    NodeSensors s;
    NodePose p;
    int16_t vx;
    int16_t vy;
    int16_t w;
    uint8_t centered;

    for (step = 0; step < NODE_CORRECT_MAX_STEPS; step++)
    {
        if (Node_ReadSensors(&s) !=
            (NODE_READ_FRONT | NODE_READ_BACK | NODE_READ_LEFT | NODE_READ_RIGHT))
        {
            Node_Stop();
            delay_ms(NODE_CORRECT_STEP_MS);
            continue;
        }

        Node_GetPoseCenter(&s, &p);
        centered = (uint8_t)(
            s.front_ok && s.back_ok && s.left_ok && s.right_ok
            && ((s.front_bits & NODE_CENTER_MASK) == NODE_CENTER_MASK)
            && ((s.back_bits & NODE_CENTER_MASK) == NODE_CENTER_MASK)
            && ((s.left_bits & NODE_CENTER_MASK) == NODE_CENTER_MASK)
            && ((s.right_bits & NODE_CENTER_MASK) == NODE_CENTER_MASK)
            && (Node_Abs16(p.lat) <= NODE_CORRECT_DEADBAND)
            && (Node_Abs16(p.lon) <= NODE_CORRECT_DEADBAND)
            && (Node_Abs16(p.yaw) <= NODE_CORRECT_DEADBAND));
        if (centered)
        {
            Node_Stop();
            return 1;
        }

        if (p.valid_count == 0)
        {
            Node_Stop();
            delay_ms(NODE_CORRECT_STEP_MS);
            continue;
        }

        vx = (int16_t)(Node_SignStep(p.lon, NODE_TRACK_DEADBAND,
                                     NODE_TRACK_LONG_SIGN) * NODE_CORRECT_SPEED);
        vy = (int16_t)(Node_SignStep(p.lat, NODE_TRACK_DEADBAND,
                                     NODE_TRACK_LAT_SIGN) * NODE_CORRECT_SPEED);
        w  = (int16_t)Node_SignStep(p.yaw, NODE_TRACK_DEADBAND,
                                    NODE_TRACK_YAW_SIGN);

        if ((vx == 0) && (vy == 0) && (w == 0))
        {
            Node_Stop();
            delay_ms(NODE_CORRECT_STEP_MS);
            continue;
        }

        Node_MoveVectorW(vx, vy, w);
        delay_ms(NODE_CORRECT_STEP_MS);
        Node_Stop();
    }

    Node_Stop();
    return 0;
}

static int8_t Node_YawSignForMode(uint8_t mode)
{
    switch (mode)
    {
        case NODE_MODE_FORWARD: return NODE_TRACK_YAW_SIGN;
        case NODE_MODE_BACK:    return NODE_BACK_YAW_SIGN;
        case NODE_MODE_LEFT:
        case NODE_MODE_RIGHT:   return NODE_SIDE_YAW_SIGN;
        default:                return 0;
    }
}

static uint8_t Node_Travel(uint8_t mode)
{
    uint16_t tick;
    NodeSensors s;
    NodePose p;
    uint8_t read_ok;
    uint8_t node_armed = 0;
    uint8_t node_hit;
    uint8_t need_read;
    int16_t vx;
    int16_t vy;
    int16_t w;

    need_read = ((mode == NODE_MODE_FORWARD) || (mode == NODE_MODE_BACK))
              ? (NODE_READ_FRONT | NODE_READ_BACK)
              : (NODE_READ_LEFT | NODE_READ_RIGHT);

    for (tick = 0; tick < NODE_TRAVEL_TIMEOUT_TICKS; tick++)
    {
        read_ok = Node_ReadSensors(&s);
        if ((read_ok & need_read) != need_read)
        {
            Node_Stop();
            return 0;
        }

        if (tick >= NODE_ARM_DELAY_TICKS) node_armed = 1;

        if (node_armed)
        {
            node_hit = 0;

            if ((mode == NODE_MODE_FORWARD) || (mode == NODE_MODE_BACK))
            {
                if ((read_ok & NODE_READ_LEFT) && s.left_ok)
                {
                    node_hit |= (uint8_t)(
                        ((s.left_bits & NODE_STOP_LOW_MASK) == NODE_STOP_LOW_MASK)
                        || ((s.left_bits & NODE_STOP_HIGH_MASK) == NODE_STOP_HIGH_MASK));
                }
                if ((read_ok & NODE_READ_RIGHT) && s.right_ok)
                {
                    node_hit |= (uint8_t)(
                        ((s.right_bits & NODE_STOP_LOW_MASK) == NODE_STOP_LOW_MASK)
                        || ((s.right_bits & NODE_STOP_HIGH_MASK) == NODE_STOP_HIGH_MASK));
                }
            }
            else
            {
                if ((read_ok & NODE_READ_FRONT) && s.front_ok)
                {
                    node_hit |= (uint8_t)(
                        ((s.front_bits & NODE_STOP_LOW_MASK) == NODE_STOP_LOW_MASK)
                        || ((s.front_bits & NODE_STOP_HIGH_MASK) == NODE_STOP_HIGH_MASK));
                }
                if ((read_ok & NODE_READ_BACK) && s.back_ok)
                {
                    node_hit |= (uint8_t)(
                        ((s.back_bits & NODE_STOP_LOW_MASK) == NODE_STOP_LOW_MASK)
                        || ((s.back_bits & NODE_STOP_HIGH_MASK) == NODE_STOP_HIGH_MASK));
                }
            }

            if (node_hit)
            {
                if (mode == NODE_MODE_FORWARD)
                    Node_MoveVector(-NODE_BRAKE_SPEED, 0);
                else if (mode == NODE_MODE_BACK)
                    Node_MoveVector(NODE_BRAKE_SPEED, 0);
                else if (mode == NODE_MODE_RIGHT)
                    Node_MoveVector(0, NODE_BRAKE_SPEED);
                else
                    Node_MoveVector(0, -NODE_BRAKE_SPEED);
                delay_ms(NODE_BRAKE_MS);
                Node_Stop();
                return 1;
            }
        }

        Node_GetPoseTravel(&s, &p, mode);
        if (p.valid_count == 0)
        {
            Node_Stop();
            delay_ms(NODE_TICK_MS);
            continue;
        }

        if (Node_Abs16(p.lat) <= NODE_TRACK_DEADBAND) p.lat = 0;
        if (Node_Abs16(p.lon) <= NODE_TRACK_DEADBAND) p.lon = 0;
        if (Node_Abs16(p.yaw) <= NODE_TRACK_DEADBAND) p.yaw = 0;

        if ((mode == NODE_MODE_FORWARD) || (mode == NODE_MODE_BACK))
        {
            vx = (int16_t)((mode == NODE_MODE_FORWARD)
                 ? NODE_CRUISE : -NODE_CRUISE);
            vy = Node_TrackChannel(p.lat, NODE_TRACK_LAT_K,
                                   NODE_TRACK_LAT_MAX, NODE_TRACK_LAT_SIGN);
        }
        else
        {
            vx = Node_TrackChannel(p.lon, NODE_TRACK_LAT_K,
                                   NODE_TRACK_LONG_MAX, NODE_TRACK_LONG_SIGN);
            vy = (int16_t)((mode == NODE_MODE_RIGHT)
                 ? -NODE_CRUISE : NODE_CRUISE);
        }

        w = Node_TrackChannel(p.yaw, NODE_TRACK_YAW_K,
                              NODE_TRACK_YAW_MAX,
                              Node_YawSignForMode(mode));

        if (Node_MoveVectorW(vx, vy, w))
        {
            Node_Stop();
            return 0;
        }

        delay_ms(NODE_TICK_MS);
    }

    Node_Stop();
    return 0;
}

static const char *Node_ModeName(uint8_t mode)
{
    switch (mode)
    {
        case NODE_MODE_FORWARD: return "FORWARD";
        case NODE_MODE_BACK:    return "BACK";
        case NODE_MODE_LEFT:    return "LEFT";
        case NODE_MODE_RIGHT:   return "RIGHT";
        default:                return "UNKNOWN";
    }
}

static void Node_IRSensorInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;

    gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOE, &gpio);
}

static void Node_RightShiftForever(void)
{
    NodeSensors s;
    NodePose p;
    uint8_t read_ok;
    int16_t vx;
    int16_t vy;
    int16_t w;

    for (;;)
    {
        read_ok = Node_ReadSensors(&s);
        Node_GetPoseTravel(&s, &p, NODE_MODE_RIGHT);

        if (((read_ok & NODE_READ_LEFT) == 0)
            || ((read_ok & NODE_READ_RIGHT) == 0)
            || (p.valid_count == 0))
        {
            Node_Stop();
            LED_RED   = NODE_LED_OFF;
            LED_GREEN = NODE_LED_OFF;
            delay_ms(NODE_TICK_MS);
            continue;
        }

        if (Node_Abs16(p.lat) <= NODE_TRACK_DEADBAND) p.lat = 0;
        if (Node_Abs16(p.lon) <= NODE_TRACK_DEADBAND) p.lon = 0;
        if (Node_Abs16(p.yaw) <= NODE_TRACK_DEADBAND) p.yaw = 0;

        vx = Node_TrackChannel(p.lon, NODE_TRACK_LAT_K,
                               NODE_TRACK_LONG_MAX, NODE_TRACK_LONG_SIGN);
        vy = (int16_t)(-NODE_CRUISE
             + Node_TrackChannel(p.lat, NODE_TRACK_LAT_K,
                                 NODE_TRACK_LAT_MAX, NODE_TRACK_LAT_SIGN));
        w = Node_TrackChannel(p.yaw, NODE_TRACK_YAW_K,
                              NODE_TRACK_YAW_MAX, NODE_SIDE_YAW_SIGN);

        if (Node_MoveVectorW(vx, vy, w))
        {
            Node_Stop();
            LED_RED   = NODE_LED_ON;
            LED_GREEN = NODE_LED_OFF;
        }
        else
        {
            LED_RED   = NODE_LED_OFF;
            LED_GREEN = NODE_LED_ON;
        }

        delay_ms(NODE_TICK_MS);
    }
}

static uint8_t Node_FrontObstacle(void)
{
    uint8_t i;
    uint8_t hit_count = 0;

    for (i = 0; i < 4; i++)
    {
        if (NODE_IR_FRONT_L || NODE_IR_FRONT_R)
        {
            if (hit_count < NODE_IR_CONFIRM) hit_count++;
        }
        else
        {
            if (hit_count > 0) hit_count--;
        }

        if (hit_count >= NODE_IR_CONFIRM) return 1;
        delay_ms(NODE_IR_POLL_MS);
    }

    return 0;
}

static void Node_Lamp(uint8_t state, uint16_t tick)
{
    switch (state)
    {
        case 0: /* 直行/居中 */
            LED_RED   = NODE_LED_OFF;
            LED_GREEN = NODE_LED_ON;
            break;
        case 1: /* 成功锁停 */
            LED_RED   = NODE_LED_ON;
            LED_GREEN = NODE_LED_OFF;
            break;
        case 2: /* 故障/超时 */
            LED_RED   = ((tick / 3) & 1) ? NODE_LED_OFF : NODE_LED_ON;
            LED_GREEN = NODE_LED_OFF;
            break;
        default:
            LED_RED   = NODE_LED_OFF;
            LED_GREEN = NODE_LED_OFF;
            break;
    }
}

/* ---------------------------------------------------------------------------
 *  七、主程序
 * ------------------------------------------------------------------------- */

int main(void)
{
    uint8_t front_raw = 0;
    uint8_t right_raw = 0;
    uint16_t tick = 0;
    uint8_t task_index;
    uint8_t mode;
    uint8_t success;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    USART1_Init(19200);

    LED_RED   = NODE_LED_OFF;
    LED_GREEN = NODE_LED_OFF;

    MOTOR_Init();
    Node_Stop();
    delay_ms(100);
    f_sw_i2c_init();
    b_sw_i2c_init();
    l_sw_i2c_init();
    r_sw_i2c_init();
    Node_IRSensorInit();
    delay_ms(100);

#if NODE_DEBUG
    printf("\r\n== NodeTest v8: fixed node task sequence ==\r\n");
    printf("NodeTest: FWD -> RIGHT -> FWD -> RIGHT -> FWD -> LEFT -> BACK\r\n");
    printf("NodeTest: cruise=%d correct_speed=%d tick=%dms\r\n",
           (int)NODE_CRUISE, (int)NODE_CORRECT_SPEED, (int)NODE_TICK_MS);
    printf("NodeTest: translation K=%d max=%d, yaw K=%d max=%d, deadband=%d\r\n",
           (int)NODE_TRACK_LAT_K, (int)NODE_TRACK_LAT_MAX,
           (int)NODE_TRACK_YAW_K, (int)NODE_TRACK_YAW_MAX,
           (int)NODE_TRACK_DEADBAND);
    printf("NodeTest: all four YB-MUX04 @0x12 cmd1\r\n");
#endif

    delay_ms(NODE_START_DELAY_MS);

    success = 1;
    for (task_index = 0; task_index < NODE_TASK_COUNT; task_index++)
    {
        mode = NODE_TASK_MODES[task_index];

#if NODE_DEBUG
        printf("NodeTest: STEP %u/%u %s START\r\n",
               (unsigned int)(task_index + 1U),
               (unsigned int)NODE_TASK_COUNT,
               Node_ModeName(mode));
#endif

        if (Node_Travel(mode) == 0)
        {
            Node_Stop();
#if NODE_DEBUG
            printf("NodeTest: STEP %u/%u %s FAILED/TIMEOUT\r\n",
                   (unsigned int)(task_index + 1U),
                   (unsigned int)NODE_TASK_COUNT,
                   Node_ModeName(mode));
#endif
            success = 0;
            break;
        }

        Node_Stop();
#if NODE_DEBUG
        printf("NodeTest: STEP %u/%u %s NODE STOP\r\n",
               (unsigned int)(task_index + 1U),
               (unsigned int)NODE_TASK_COUNT,
               Node_ModeName(mode));
#endif
        delay_ms(NODE_INERTIA_SETTLE_MS);
    }

    Node_Stop();

    for (;;)
    {
        Node_Lamp(success ? 1 : 2, tick);
        if ((tick % 25) == 0 && success == 0)
        {
            if (Node_ReadBoth(&front_raw, &right_raw))
            {
#if NODE_DEBUG
                printf("NodeTest: end raw F=0x%02X R=0x%02X\r\n",
                       (unsigned int)front_raw, (unsigned int)right_raw);
#endif
            }
        }
        tick++;
        delay_ms(200);
    }
}

/* 工程清单包含四块灰阶驱动，它们链接时需要 reverse()。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (uint8_t)((x >> 4) | (x << 4));
}
