/* ============================================================================
 * A车程序 FinalProgram_A —— 去程 BFS、记录路线、投递、返程动态避障
 *
 * 【场地】
 *   节点范围 X=0..7, Y=0..4；车头固定朝 +Y。
 *   红方：(7,0) -> (0,4)
 *   蓝方：(0,0) -> (7,4)
 *
 * 【任务流程】
 *   去程：节点停车 -> 八路红外扫描 -> 更新障碍地图
 *         -> BFS 计算下一节点 -> 移动并记录实际路线
 *   终点：执行投递任务占位
 *   返程：优先逆序沿原路返回；下一节点持续被占用时记为固定障碍，
 *         再用 BFS 找回家路线，并在等长路径中优先选择原路节点
 *
 * 【底盘与循迹】
 *   直接参考 NodeTest：
 *   - 前进、后退、左移、右移；
 *   - 平移和旋转纠偏；
 *   - 23/67 节点检测；
 *   - 反向制动 400 ms。
 *
 * 【灰阶】
 *   前：PB13=SCL，PB14=SDA
 *   后：PD1 =SCL，PD2 =SDA
 *   左：PC8 =SCL，PC9 =SDA
 *   右：PC3 =SCL，PC2 =SDA
 *   四块均为亚博 YB-MUX04-2.0，地址 0x12，数据寄存器 0x30。
 * ========================================================================== */

#include "main.h"

/* ---------------------------------------------------------------------------
 *  一、可调参数
 * ------------------------------------------------------------------------- */

/* 节点间直行速度，单位与 MoveTest 的 0x33 一致。 */
#define NODE_CRUISE                 17

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
#define NODE_TRACK_LAT_MAX          3
#define NODE_TRACK_LAT_SIGN         (+1)
#define NODE_TRACK_LONG_MAX         3
#define NODE_TRACK_LONG_SIGN        (+1)
#define NODE_TRACK_YAW_K            8
#define NODE_TRACK_YAW_MAX          2
#define NODE_TRACK_YAW_SIGN         (+1)  /* forward, restored non-reversed */
#define NODE_BACK_YAW_SIGN          (+1)  /* backward, same sign after front/back remap */
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
#define NODE_STOP_MASK_23           GREY6_STOP_MASK_23
#define NODE_STOP_MASK_67           GREY6_STOP_MASK_67
#define NODE_ARM_DELAY_TICKS        25
#define NODE_MID_IR_CHECK_INTERVAL_MS  200U
#define NODE_MID_IR_CHECK_COUNT         5U
#define NODE_MID_IR_CHECK_INTERVAL_TICKS \
    (NODE_MID_IR_CHECK_INTERVAL_MS / NODE_TICK_MS)
#define NODE_RETRACE_ARM_TICKS      2U

/* 0 = skip post-stop centering, 1 = keep Node_Center() enabled. */
#define NODE_POST_STOP_CENTER        0
#define NODE_BRAKE_SPEED            4
#define NODE_BRAKE_MS               400

#define NODE_MODE_FORWARD            0
#define NODE_MODE_BACK               1
#define NODE_MODE_LEFT               2
#define NODE_MODE_RIGHT              3

/* 队伍默认红方；运行时通过 PD10 在红方和蓝方之间切换。 */
#define FINAL_TEAM_RED               0
#define FINAL_TEAM_BLUE              1
#define FINAL_DEFAULT_TEAM           FINAL_TEAM_RED
#define FINAL_TEAM_RED_DISPLAY       0U
#define FINAL_TEAM_BLUE_DISPLAY      1111U
#define FINAL_RUN_DISPLAY            1234U
#define FINAL_TEAM_KEY_CONFIRM       3U
#define FINAL_TEAM_KEY_HIT           (!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))

/* TCRT5000 start sensor on PD12. Idle is high, trigger is low. */
#define FINAL_START_IR_HIT           (!START_FLAG)
#define FINAL_START_IR_CLEAR         (START_FLAG)
#define FINAL_START_IR_CONFIRM       3U
#define FINAL_START_IR_POLL_MS       20U

/* 1 = 只测试连续前进循迹；其他 BFS、红外、停车、舵机任务全部不执行。 */
#define FINAL_FORWARD_ONLY_TEST      0
#define FINAL_FORWARD_TRANSLATION_ENABLE 1
#define FINAL_FORWARD_YAW_ENABLE     1

#define FINAL_GRID_W                 8U
#define FINAL_GRID_H                 5U
#define FINAL_GRID_NODES             (FINAL_GRID_W * FINAL_GRID_H)
#define FINAL_MAX_STEPS              64U
#define FINAL_ROUTE_CAPACITY         (FINAL_MAX_STEPS + 1U)
#define FINAL_RETURN_RECHECK_MS      2000U
#define FINAL_SERVO_DELIVER_PE5_US   800U
#define FINAL_SERVO_DELIVER_PE6_US   1700U
#define FINAL_SERVO_MID_US           1250U
#define FINAL_SERVO_HOLD_PE5_US      1700U
#define FINAL_SERVO_HOLD_PE6_US      800U
#define FINAL_SERVO_STEP_MS          1500U

#define FINAL_SERVO_STATE_DELIVER    0U
#define FINAL_SERVO_STATE_MID        1U
#define FINAL_SERVO_STATE_HOLD       2U

static uint8_t Final_StartX = 7U;
static uint8_t Final_StartY = 0U;
static uint8_t Final_GoalX = 0U;
static uint8_t Final_GoalY = 4U;
static const char *Final_TeamName = "RED";

static void Final_ConfigureTeam(uint8_t team)
{
    if (team == FINAL_TEAM_BLUE)
    {
        Final_StartX = 0U;
        Final_StartY = 0U;
        Final_GoalX = 7U;
        Final_GoalY = 4U;
        Final_TeamName = "BLUE";
    }
    else
    {
        Final_StartX = 7U;
        Final_StartY = 0U;
        Final_GoalX = 0U;
        Final_GoalY = 4U;
        Final_TeamName = "RED";
    }
}

#define FINAL_START_X Final_StartX
#define FINAL_START_Y Final_StartY
#define FINAL_GOAL_X  Final_GoalX
#define FINAL_GOAL_Y  Final_GoalY
#define FINAL_TEAM_NAME Final_TeamName

/* 1 = 已确认的固定障碍节点。未知节点按可通过处理。 */
static uint8_t Final_Blocked[FINAL_GRID_W][FINAL_GRID_H];

static void Final_ClearBlockedMap(void)
{
    uint8_t x;
    uint8_t y;

    for (x = 0U; x < FINAL_GRID_W; x++)
        for (y = 0U; y < FINAL_GRID_H; y++)
            Final_Blocked[x][y] = 0U;
}


/* 去程实际路线节点。返程 BFS 在等长路径中优先选择这些节点。 */
static uint8_t Final_Preferred[FINAL_GRID_W][FINAL_GRID_H];

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
#define NODE_IR_BACK_L  NODE_IR_READ(GPIOB, GPIO_Pin_7)
#define NODE_IR_BACK_R  NODE_IR_READ(GPIOE, GPIO_Pin_4)
#define NODE_IR_LEFT_U  NODE_IR_READ(GPIOD, GPIO_Pin_14)
#define NODE_IR_LEFT_D  NODE_IR_READ(GPIOB, GPIO_Pin_8)
#define NODE_IR_RIGHT_U NODE_IR_READ(GPIOE, GPIO_Pin_11)
#define NODE_IR_RIGHT_D NODE_IR_READ(GPIOE, GPIO_Pin_3)

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
#define NODE_FRONT_RIGHT_MASK       GREY6_STOP_MASK_23
#define NODE_FRONT_RIGHT_HALF       0x1E
#define NODE_FRONT_LEFT_MASK        GREY6_STOP_MASK_67
#define NODE_FRONT_LEFT_HALF        0x78

/* 只作用于 Node_Center() 的左右修正，不改变循迹或传感器识别。 */
#define NODE_CORRECT_LATERAL_SIGN   (-1)

#define NODE_RIGHT_FORWARD_MASK     GREY6_STOP_MASK_23
#define NODE_RIGHT_FORWARD_HALF     0x1E
#define NODE_RIGHT_BACK_MASK        GREY6_STOP_MASK_67
#define NODE_RIGHT_BACK_HALF        0x78

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
#define NODE_YB_STATE_CMD 0x30U

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
    for (i = 2; i < 7; i++)
    {
        if (((x >> i) & 1U) != ((x >> (i - 1)) & 1U)) t++;
    }
    return t;
}

static uint8_t Node_Valid(uint8_t raw)
{
    uint8_t pc;
    uint8_t active = (uint8_t)(raw & GREY6_ACTIVE_MASK);

    pc = Node_Popcount(active);
    if ((pc == 0) || (pc == GREY6_BLACK_COUNT)) return 0;
    if (Node_Transitions(active) > 2) return 0;
    return 1;
}

/* Same conversion as StraightTest front YB-MUX04-2.0. */
static uint8_t Node_YBBits(uint8_t raw)
{
    uint8_t v = reverse(raw);
    v = (uint8_t)((v >> 1) | (v << 7));
    return (uint8_t)((~v) & GREY6_ACTIVE_MASK);
}

static uint8_t Node_EdgeMask(uint8_t bits)
{
    return (uint8_t)(bits & GREY6_ACTIVE_MASK);
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
    static const int8_t W[8] = { 0, 7, 4, 1, -1, -4, -7, 0 };
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
    static const int8_t FRONT_W[8] = { 0, +7, +4, +1, -1, -4, -7, 0 };
    static const int8_t BACK_W[8]  = { 0, +7, +4, +1, -1, -4, -7, 0 };
    return Node_Weighted(bits, back ? BACK_W : FRONT_W);
}

static int16_t Node_TrackYaw(uint8_t bits)
{
    static const int8_t YAW_W[8] = { 0, +7, +4, +1, -1, -4, -7, 0 };
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
        p->lat = (int16_t)(p->lat - Node_TrackLat(s->back_ctrl, 1));
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

    /* Front/back modules are mirrored: lateral subtracts rear position, yaw sums. */
    if (s->front_ok && s->back_ok)
        p->yaw = (int16_t)(p->yaw
                + Node_TrackYaw(s->front_ctrl)
                + Node_TrackYaw(s->back_ctrl));
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
            p->lat = (int16_t)(p->lat - Node_TrackLat(s->back_ctrl, 1));
            p->valid_count++;
        }
        if (s->front_ok && s->back_ok)
            p->yaw = (int16_t)(Node_TrackYaw(s->front_ctrl)
                             + Node_TrackYaw(s->back_ctrl));
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

static uint8_t Node_StopPairHit(uint8_t bits)
{
    return (uint8_t)(
        (((bits & NODE_STOP_MASK_23) == NODE_STOP_MASK_23)
      || ((bits & NODE_STOP_MASK_67) == NODE_STOP_MASK_67)));
}

static void Final_ReadIRSides(uint8_t x, uint8_t y,
                              uint8_t *front_out,
                              uint8_t *back_out,
                              uint8_t *left_out,
                              uint8_t *right_out);
static uint8_t Final_IrBlocksMode(uint8_t mode,
                                  uint8_t front,
                                  uint8_t back,
                                  uint8_t left,
                                  uint8_t right);
static const char *Node_ModeName(uint8_t mode);
static uint8_t Node_ReverseMode(uint8_t mode);
static void Node_BrakeForMode(uint8_t mode);

static uint8_t Node_TravelCore(uint8_t mode,
                               uint8_t start_x,
                               uint8_t start_y,
                               uint8_t check_mid_obstacle,
                               uint8_t arm_delay_ticks)
{
    uint16_t tick;
    NodeSensors s;
    NodePose p;
    uint8_t read_ok;
    uint8_t node_armed = 0;
    uint8_t node_hit;
    uint8_t need_read;
    uint8_t front;
    uint8_t back;
    uint8_t left;
    uint8_t right;
    uint8_t mid_checks_done = 0U;
    uint16_t next_mid_check_tick = NODE_MID_IR_CHECK_INTERVAL_TICKS;
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

        if (check_mid_obstacle &&
            (mid_checks_done < NODE_MID_IR_CHECK_COUNT) &&
            (tick >= next_mid_check_tick))
        {
            uint8_t check_no = (uint8_t)(mid_checks_done + 1U);
            uint16_t check_ms = (uint16_t)(tick * NODE_TICK_MS);

            mid_checks_done++;
            next_mid_check_tick =
                (uint16_t)(next_mid_check_tick + NODE_MID_IR_CHECK_INTERVAL_TICKS);
            Final_ReadIRSides(start_x, start_y,
                              &front, &back, &left, &right);
            if (Final_IrBlocksMode(mode, front, back, left, right))
            {
                uint8_t reverse_mode = Node_ReverseMode(mode);

#if NODE_DEBUG
                printf("Final: MID OBSTACLE check=%u time=%ums mode=%s, return to (%u,%u)\r\n",
                       (unsigned int)check_no,
                       (unsigned int)check_ms,
                       Node_ModeName(mode),
                       (unsigned int)start_x, (unsigned int)start_y);
#endif

                Node_BrakeForMode(mode);
                if ((reverse_mode == 0xFFU) ||
                    (Node_TravelCore(reverse_mode, start_x, start_y,
                                     0U, NODE_RETRACE_ARM_TICKS) == 0U))
                    return 0U;

                return 2U;
            }
        }

        if (tick >= arm_delay_ticks) node_armed = 1;

        if (node_armed)
        {
            node_hit = 0;

            if ((mode == NODE_MODE_FORWARD) || (mode == NODE_MODE_BACK))
            {
                if ((read_ok & NODE_READ_LEFT) && s.left_ok)
                    node_hit |= Node_StopPairHit(s.left_bits);
                if ((read_ok & NODE_READ_RIGHT) && s.right_ok)
                    node_hit |= Node_StopPairHit(s.right_bits);
            }
            else
            {
                if ((read_ok & NODE_READ_FRONT) && s.front_ok)
                    node_hit |= Node_StopPairHit(s.front_bits);
                if ((read_ok & NODE_READ_BACK) && s.back_ok)
                    node_hit |= Node_StopPairHit(s.back_bits);
            }

            if (node_hit)
            {
                Node_BrakeForMode(mode);
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


static uint8_t Node_Travel(uint8_t mode,
                           uint8_t start_x,
                           uint8_t start_y)
{
    return Node_TravelCore(mode, start_x, start_y,
                           1U, NODE_ARM_DELAY_TICKS);
}

static uint8_t Node_TravelNoMid(uint8_t mode,
                                uint8_t start_x,
                                uint8_t start_y)
{
    return Node_TravelCore(mode, start_x, start_y,
                           0U, NODE_ARM_DELAY_TICKS);
}

static uint8_t Node_ReverseMode(uint8_t mode)
{
    if (mode == NODE_MODE_FORWARD) return NODE_MODE_BACK;
    if (mode == NODE_MODE_BACK) return NODE_MODE_FORWARD;
    if (mode == NODE_MODE_LEFT) return NODE_MODE_RIGHT;
    if (mode == NODE_MODE_RIGHT) return NODE_MODE_LEFT;
    return 0xFFU;
}

static void Node_BrakeForMode(uint8_t mode)
{
    if (mode == NODE_MODE_FORWARD)
        Node_MoveVector(-NODE_BRAKE_SPEED, 0);
    else if (mode == NODE_MODE_BACK)
        Node_MoveVector(NODE_BRAKE_SPEED, 0);
    else if (mode == NODE_MODE_RIGHT)
        Node_MoveVector(0, NODE_BRAKE_SPEED);
    else if (mode == NODE_MODE_LEFT)
        Node_MoveVector(0, -NODE_BRAKE_SPEED);
    delay_ms(NODE_BRAKE_MS);
    Node_Stop();
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

typedef struct
{
    uint8_t count;
    uint8_t on;
} Node_IrSide;

static uint8_t Final_CellIndex(uint8_t x, uint8_t y)
{
    return (uint8_t)((x * FINAL_GRID_H) + y);
}

static uint8_t Node_IrUpdate(Node_IrSide *side, uint8_t hit)
{
    if (hit)
    {
        if (side->count < 255U) side->count++;
    }
    else
    {
        side->count = 0;
    }

    side->on = (uint8_t)(side->count >= NODE_IR_CONFIRM);
    return side->on;
}

static void Final_ReadIRSides(uint8_t x, uint8_t y,
                              uint8_t *front_out,
                              uint8_t *back_out,
                              uint8_t *left_out,
                              uint8_t *right_out)
{
    Node_IrSide front_s = { 0, 0 };
    Node_IrSide back_s  = { 0, 0 };
    Node_IrSide left_s  = { 0, 0 };
    Node_IrSide right_s = { 0, 0 };
    uint8_t i;

    for (i = 0; i < 3U; i++)
    {
        Node_IrUpdate(&front_s,
                      (uint8_t)(NODE_IR_FRONT_L || NODE_IR_FRONT_R));
        Node_IrUpdate(&back_s,
                      (uint8_t)(NODE_IR_BACK_L || NODE_IR_BACK_R));
        Node_IrUpdate(&left_s,
                      (uint8_t)(NODE_IR_LEFT_U || NODE_IR_LEFT_D));
        Node_IrUpdate(&right_s,
                      (uint8_t)(NODE_IR_RIGHT_U || NODE_IR_RIGHT_D));
        delay_ms(NODE_IR_POLL_MS);
    }

    *front_out = front_s.on;
    *back_out  = back_s.on;
    *left_out  = left_s.on;
    *right_out = right_s.on;

#if NODE_DEBUG
    printf("Final: node=(%u,%u) IR F=%u B=%u L=%u R=%u\r\n",
           (unsigned int)x, (unsigned int)y,
           (unsigned int)*front_out, (unsigned int)*back_out,
           (unsigned int)*left_out, (unsigned int)*right_out);
#endif
}

static void Final_UpdateBlockedMap(uint8_t x, uint8_t y,
                                   uint8_t front,
                                   uint8_t back,
                                   uint8_t left,
                                   uint8_t right)
{
    if (front && ((y + 1U) < FINAL_GRID_H)) Final_Blocked[x][y + 1U] = 1U;
    if (back  && (y > 0U))                  Final_Blocked[x][y - 1U] = 1U;
    if (left  && (x > 0U))                  Final_Blocked[x - 1U][y] = 1U;
    if (right && ((x + 1U) < FINAL_GRID_W)) Final_Blocked[x + 1U][y] = 1U;

    Final_Blocked[x][y] = 0U;
}

static uint8_t Final_IrHasUnknownObstacle(uint8_t x, uint8_t y,
                                           uint8_t front,
                                           uint8_t back,
                                           uint8_t left,
                                           uint8_t right)
{
    if (front && ((y + 1U) < FINAL_GRID_H)
        && !Final_Blocked[x][y + 1U]) return 1U;
    if (back && (y > 0U)
        && !Final_Blocked[x][y - 1U]) return 1U;
    if (left && (x > 0U)
        && !Final_Blocked[x - 1U][y]) return 1U;
    if (right && ((x + 1U) < FINAL_GRID_W)
        && !Final_Blocked[x + 1U][y]) return 1U;
    return 0U;
}

static uint8_t Final_ModeForStep(uint8_t x, uint8_t y,
                                 uint8_t nx, uint8_t ny)
{
    if ((nx == x) && (ny == (uint8_t)(y + 1U))) return NODE_MODE_FORWARD;
    if ((nx == x) && (ny == (uint8_t)(y - 1U))) return NODE_MODE_BACK;
    if ((ny == y) && (nx == (uint8_t)(x - 1U))) return NODE_MODE_LEFT;
    if ((ny == y) && (nx == (uint8_t)(x + 1U))) return NODE_MODE_RIGHT;
    return 0xFFU;
}

static uint8_t Final_ModeTargetKnownBlocked(uint8_t x, uint8_t y,
                                             uint8_t mode)
{
    uint8_t nx = x;
    uint8_t ny = y;

    if (mode == NODE_MODE_FORWARD) ny++;
    else if (mode == NODE_MODE_BACK) ny--;
    else if (mode == NODE_MODE_LEFT) nx--;
    else if (mode == NODE_MODE_RIGHT) nx++;
    else return 0U;

    if ((nx >= FINAL_GRID_W) || (ny >= FINAL_GRID_H)) return 0U;
    return Final_Blocked[nx][ny];
}

static uint8_t Final_StepBackHistory(const uint8_t *path_x,
                                     const uint8_t *path_y,
                                     uint8_t *path_len,
                                     uint8_t *current_x,
                                     uint8_t *current_y)
{
    uint8_t mode;
    int16_t i;

    if (*path_len < 2U) return 0U;
    i = (int16_t)(*path_len - 2U);
    mode = Final_ModeForStep(*current_x, *current_y,
                             path_x[i], path_y[i]);
    if (mode == 0xFFU) return 0U;
    if (Node_TravelNoMid(mode, *current_x, *current_y) == 0U)
        return 0U;

    *current_x = path_x[i];
    *current_y = path_y[i];
    (*path_len)--;
    return 1U;
}

static uint8_t Final_IrBlocksMode(uint8_t mode,
                                   uint8_t front,
                                   uint8_t back,
                                   uint8_t left,
                                   uint8_t right)
{
    if (mode == NODE_MODE_FORWARD) return front;
    if (mode == NODE_MODE_BACK)    return back;
    if (mode == NODE_MODE_LEFT)    return left;
    if (mode == NODE_MODE_RIGHT)   return right;
    return 1U;
}

static void Final_MarkBlockedForMode(uint8_t x, uint8_t y, uint8_t mode)
{
    uint8_t bx = x;
    uint8_t by = y;

    if (mode == NODE_MODE_FORWARD) by++;
    else if (mode == NODE_MODE_BACK) by--;
    else if (mode == NODE_MODE_LEFT) bx--;
    else if (mode == NODE_MODE_RIGHT) bx++;
    else return;

    if ((bx < FINAL_GRID_W) && (by < FINAL_GRID_H))
    {
        Final_Blocked[bx][by] = 1U;
#if NODE_DEBUG
        printf("Final: FIXED OBSTACLE (%u,%u), from node (%u,%u)\r\n",
               (unsigned int)bx, (unsigned int)by,
               (unsigned int)x, (unsigned int)y);
#endif
    }
}

static uint8_t Final_ReturnObstaclePersists(uint8_t x,
                                             uint8_t y,
                                             uint8_t mode)
{
    uint8_t front;
    uint8_t back;
    uint8_t left;
    uint8_t right;
    uint8_t blocked;

    if (Final_ModeTargetKnownBlocked(x, y, mode))
    {
#if NODE_DEBUG
        printf("Final: RETURN KNOWN OBSTACLE (%u,%u) mode=%s, no wait\r\n",
               (unsigned int)x, (unsigned int)y,
               Node_ModeName(mode));
#endif
        return 1U;
    }

    delay_ms(FINAL_RETURN_RECHECK_MS);
    Final_ReadIRSides(x, y, &front, &back, &left, &right);
    blocked = Final_IrBlocksMode(mode, front, back, left, right);

#if NODE_DEBUG
    printf("Final: RETURN RECHECK (%u,%u) mode=%s blocked=%u\r\n",
           (unsigned int)x, (unsigned int)y,
           Node_ModeName(mode), (unsigned int)blocked);
#endif

    return blocked;
}

static uint8_t Final_BfsNext(uint8_t sx, uint8_t sy,
                             uint8_t gx, uint8_t gy,
                             uint8_t *next_x, uint8_t *next_y)
{
    static const int8_t DX[4] = {  0, +1, -1,  0 };
    static const int8_t DY[4] = { +1,  0,  0, -1 };
    int16_t dist[FINAL_GRID_NODES];
    uint8_t queue[FINAL_GRID_NODES];
    uint8_t head = 0U;
    uint8_t tail = 0U;
    uint8_t i;
    uint8_t cur;
    uint8_t start = Final_CellIndex(sx, sy);
    uint8_t goal  = Final_CellIndex(gx, gy);
    uint8_t best = 0xFFU;
    uint8_t best_score = 2U;

    if (Final_Blocked[sx][sy] || Final_Blocked[gx][gy]) return 0U;
    if (start == goal) return 0U;

    for (i = 0; i < FINAL_GRID_NODES; i++) dist[i] = -1;

    /* 从返程目标倒推最短距离。 */
    dist[goal] = 0;
    queue[tail++] = goal;

    while (head < tail)
    {
        cur = queue[head++];

        for (i = 0; i < 4U; i++)
        {
            int8_t nx = (int8_t)((int8_t)(cur / FINAL_GRID_H) + DX[i]);
            int8_t ny = (int8_t)((int8_t)(cur % FINAL_GRID_H) + DY[i]);
            uint8_t ni;

            if ((nx < 0) || (nx >= (int8_t)FINAL_GRID_W)) continue;
            if ((ny < 0) || (ny >= (int8_t)FINAL_GRID_H)) continue;
            if (Final_Blocked[(uint8_t)nx][(uint8_t)ny]) continue;

            ni = Final_CellIndex((uint8_t)nx, (uint8_t)ny);
            if (dist[ni] != -1) continue;

            dist[ni] = (int16_t)(dist[cur] + 1);
            queue[tail++] = ni;
        }
    }

    if (dist[start] <= 0) return 0U;

    /* 在所有能缩短距离的相邻节点中，优先选择去程记录过的节点。 */
    for (i = 0; i < 4U; i++)
    {
        int8_t nx = (int8_t)((int8_t)sx + DX[i]);
        int8_t ny = (int8_t)((int8_t)sy + DY[i]);
        uint8_t ni;
        uint8_t score;

        if ((nx < 0) || (nx >= (int8_t)FINAL_GRID_W)) continue;
        if ((ny < 0) || (ny >= (int8_t)FINAL_GRID_H)) continue;
        if (Final_Blocked[(uint8_t)nx][(uint8_t)ny]) continue;

        ni = Final_CellIndex((uint8_t)nx, (uint8_t)ny);
        if (dist[ni] != (int16_t)(dist[start] - 1)) continue;

        score = Final_Preferred[(uint8_t)nx][(uint8_t)ny] ? 0U : 1U;
        if (score < best_score)
        {
            best = ni;
            best_score = score;
        }
    }

    if (best == 0xFFU) return 0U;

    *next_x = (uint8_t)(best / FINAL_GRID_H);
    *next_y = (uint8_t)(best % FINAL_GRID_H);
    return 1U;
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

static uint16_t Final_ServoClamp(uint16_t pulse)
{
    if (pulse < FINAL_SERVO_DELIVER_PE5_US) return FINAL_SERVO_DELIVER_PE5_US;
    if (pulse > FINAL_SERVO_HOLD_PE5_US) return FINAL_SERVO_HOLD_PE5_US;
    return pulse;
}

static void Final_ServoSetPulses(uint16_t pe5_us, uint16_t pe6_us)
{
    TIM_SetCompare1(TIM9, Final_ServoClamp(pe5_us));
    TIM_SetCompare2(TIM9, Final_ServoClamp(pe6_us));
}

static void Final_ServoApplyState(uint8_t state)
{
    if (state == FINAL_SERVO_STATE_DELIVER)
    {
        Final_ServoSetPulses(FINAL_SERVO_DELIVER_PE5_US,
                             FINAL_SERVO_DELIVER_PE6_US);
    }
    else if (state == FINAL_SERVO_STATE_MID)
    {
        Final_ServoSetPulses(FINAL_SERVO_MID_US, FINAL_SERVO_MID_US);
    }
    else
    {
        Final_ServoSetPulses(FINAL_SERVO_HOLD_PE5_US,
                             FINAL_SERVO_HOLD_PE6_US);
    }
}

static const char *Final_ServoStateName(uint8_t state)
{
    if (state == FINAL_SERVO_STATE_DELIVER) return "DELIVER";
    if (state == FINAL_SERVO_STATE_HOLD)    return "HOLD";
    return "MID";
}

static void Final_ServoRunState(uint8_t state)
{
    Final_ServoApplyState(state);
#if NODE_DEBUG
    printf("Final: SERVO %s\r\n", Final_ServoStateName(state));
#endif
    delay_ms(FINAL_SERVO_STEP_MS);
}

static void Final_ServoRelease(void)
{
    GPIO_InitTypeDef gpio;

    TIM_SetCompare1(TIM9, 0U);
    TIM_SetCompare2(TIM9, 0U);
    TIM_Cmd(TIM9, DISABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Pin  = GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &gpio);

#if NODE_DEBUG
    printf("Final: SERVO RELEASE FOR RETURN\r\n");
#endif
}

static void Final_ServoInit(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &gpio);

    TIM_TimeBaseStructInit(&time_base);
    time_base.TIM_Prescaler     = 168U - 1U;
    time_base.TIM_Period        = 20000U - 1U;
    time_base.TIM_CounterMode   = TIM_CounterMode_Up;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9, &time_base);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = FINAL_SERVO_MID_US;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM9, &oc);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM9, &oc);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);

    Final_ServoApplyState(FINAL_SERVO_STATE_HOLD);
}

static uint8_t Final_DeliveryTask(void)
{
    LED_RED   = NODE_LED_ON;
    LED_GREEN = NODE_LED_OFF;

    Final_ServoInit();
#if NODE_DEBUG
    printf("Final: SERVO INIT AT GOAL, HOLD PE5=%u PE6=%u\r\n",
           (unsigned int)FINAL_SERVO_HOLD_PE5_US,
           (unsigned int)FINAL_SERVO_HOLD_PE6_US);
#endif

    Final_ServoRunState(FINAL_SERVO_STATE_HOLD);
    Final_ServoRunState(FINAL_SERVO_STATE_MID);
    Final_ServoRunState(FINAL_SERVO_STATE_DELIVER);
    Final_ServoRunState(FINAL_SERVO_STATE_MID);
    Final_ServoRunState(FINAL_SERVO_STATE_HOLD);
    Final_ServoRelease();

    LED_RED   = NODE_LED_OFF;
    LED_GREEN = NODE_LED_ON;
    return 1U;
}

static void Final_PrintPath(const char *name,
                            const uint8_t *path_x,
                            const uint8_t *path_y,
                            uint8_t path_len)
{
#if NODE_DEBUG
    uint8_t i;

    printf("Final: %s PATH nodes=%u\r\n", name, (unsigned int)path_len);
    for (i = 0; i < path_len; i++)
    {
        printf("Final: %s[%u]=(%u,%u)\r\n",
               name, (unsigned int)i,
               (unsigned int)path_x[i],
               (unsigned int)path_y[i]);
    }
#else
    (void)name;
    (void)path_x;
    (void)path_y;
    (void)path_len;
#endif
}

static void Final_ShowTeamSelection(uint8_t team)
{
    if (team == FINAL_TEAM_BLUE)
    {
        LED_RED = NODE_LED_OFF;
        LED_GREEN = NODE_LED_ON;
        TUBES_SetValue(FINAL_TEAM_BLUE_DISPLAY);
    }
    else
    {
        LED_RED = NODE_LED_ON;
        LED_GREEN = NODE_LED_OFF;
        TUBES_SetValue(FINAL_TEAM_RED_DISPLAY);
    }
    TUBES_Display();
}

static uint8_t Final_WaitForStartSensor(void)
{
    GPIO_InitTypeDef button_gpio;
    uint8_t team = FINAL_DEFAULT_TEAM;
    uint8_t armed = 0U;
    uint8_t count = 0U;
    uint8_t hit;
    uint8_t button_level;
    uint8_t button_stable;
    uint8_t button_count = 0U;
    uint16_t tick = 0U;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    button_gpio.GPIO_Mode = GPIO_Mode_IN;
    button_gpio.GPIO_Pin = GPIO_Pin_10;
    button_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    button_gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &button_gpio);
    button_stable = (uint8_t)(FINAL_TEAM_KEY_HIT ? 1U : 0U);

    Node_Stop();
    Final_ShowTeamSelection(team);

#if NODE_DEBUG
    printf("Final: waiting TCRT5000 start on PD12\r\n");
#endif

    for (;;)
    {
        button_level = (uint8_t)(FINAL_TEAM_KEY_HIT ? 1U : 0U);
        if (button_level != button_stable)
        {
            if (button_count < FINAL_TEAM_KEY_CONFIRM) button_count++;
            if (button_count >= FINAL_TEAM_KEY_CONFIRM)
            {
                button_stable = button_level;
                button_count = 0U;
                if (button_level)
                {
                    team = (uint8_t)((team == FINAL_TEAM_RED)
                         ? FINAL_TEAM_BLUE : FINAL_TEAM_RED);
                    Final_ShowTeamSelection(team);
                }
            }
        }
        else
        {
            button_count = 0U;
        }

        hit = (uint8_t)(FINAL_START_IR_HIT ? 1U : 0U);

        if (FINAL_START_IR_CLEAR) armed = 1U;
        if (armed && hit)
        {
            if (count < FINAL_START_IR_CONFIRM) count++;
            if (count >= FINAL_START_IR_CONFIRM) break;
        }
        else
        {
            count = 0U;
        }

        /* Keep showing the selected team until TCRT5000 starts. */
        Final_ShowTeamSelection(team);
        tick++;
        delay_ms(FINAL_START_IR_POLL_MS);
    }

    Node_Stop();
    LED_RED   = NODE_LED_OFF;
    LED_GREEN = NODE_LED_ON;
    TUBES_SetValue(FINAL_RUN_DISPLAY);
    TUBES_Display();

#if NODE_DEBUG
    printf("Final: TCRT5000 start confirmed after %u polls, team=%s\r\n",
           (unsigned int)tick,
           (team == FINAL_TEAM_BLUE) ? "BLUE" : "RED");
#endif

    return team;
}

static void Final_ForwardOnlyLoop(void)
{
    NodeSensors sensors;
    NodePose pose;
    uint16_t tick = 0U;
    int16_t vx;
    int16_t vy;
    int16_t w;

    printf("Final: FORWARD-ONLY TRACK TEST\r\n");

    for (;;)
    {
        Node_ReadSensors(&sensors);
        Node_GetPoseTravel(&sensors, &pose, NODE_MODE_FORWARD);

        if (pose.valid_count == 0U)
        {
            Node_Stop();
            LED_RED   = NODE_LED_ON;
            LED_GREEN = NODE_LED_OFF;
            delay_ms(NODE_TICK_MS);
            tick++;
            continue;
        }

        if (Node_Abs16(pose.lat) <= NODE_TRACK_DEADBAND) pose.lat = 0;
        if (Node_Abs16(pose.yaw) <= NODE_TRACK_DEADBAND) pose.yaw = 0;

        vx = NODE_CRUISE;
#if FINAL_FORWARD_TRANSLATION_ENABLE
        vy = Node_TrackChannel(pose.lat, NODE_TRACK_LAT_K,
                               NODE_TRACK_LAT_MAX, NODE_TRACK_LAT_SIGN);
#else
        vy = 0;
#endif
#if FINAL_FORWARD_YAW_ENABLE
        w  = Node_TrackChannel(pose.yaw, NODE_TRACK_YAW_K,
                               NODE_TRACK_YAW_MAX,
                               Node_YawSignForMode(NODE_MODE_FORWARD));
#else
        w = 0;
#endif

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

#if NODE_DEBUG
        if ((tick % NODE_PRINT_EVERY) == 0U)
        {
            printf("Fwd: F=%02X B=%02X ok=%u/%u yaw=%d lat=%d vy=%d w=%d\r\n",
                   (unsigned int)sensors.front_bits,
                   (unsigned int)sensors.back_bits,
                   (unsigned int)sensors.front_ok,
                   (unsigned int)sensors.back_ok,
                   (int)pose.yaw, (int)pose.lat,
                   (int)vy, (int)w);
        }
#endif

        tick++;
        delay_ms(NODE_TICK_MS);
    }
}

int main(void)
{
    uint8_t selected_team;
    uint8_t current_x = 0U;
    uint8_t current_y = 0U;
    uint8_t next_x;
    uint8_t next_y;
    uint8_t mode;
    uint8_t outbound_steps = 0U;
    uint8_t route_len = 1U;
    uint8_t return_len = 1U;
    uint8_t return_steps = 0U;
    uint8_t success = 0U;
    uint8_t travel_result;
    uint8_t front;
    uint8_t back;
    uint8_t left;
    uint8_t right;
    uint8_t x;
    uint8_t y;
    uint16_t tick = 0U;
    uint8_t route_x[FINAL_ROUTE_CAPACITY];
    uint8_t route_y[FINAL_ROUTE_CAPACITY];
    uint8_t return_x[FINAL_ROUTE_CAPACITY];
    uint8_t return_y[FINAL_ROUTE_CAPACITY];

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();
    USART1_Init(19200);
    TUBES_Driver_Init();
    TUBES_SetBrightness(6U);
    TUBES_Clear();
    INFRAED_Init();

    LED_RED   = NODE_LED_OFF;
    LED_GREEN = NODE_LED_OFF;

    MOTOR_Init();
    Node_Stop();
    delay_ms(100);

    f_sw_i2c_init();
    b_sw_i2c_init();
    l_sw_i2c_init();
    r_sw_i2c_init();
#if !FINAL_FORWARD_ONLY_TEST
    Node_IRSensorInit();
#endif
    delay_ms(100);

    selected_team = Final_WaitForStartSensor();
    Final_ConfigureTeam(selected_team);
    current_x = FINAL_START_X;
    current_y = FINAL_START_Y;

#if FINAL_FORWARD_ONLY_TEST
    Final_ForwardOnlyLoop();
#endif

    for (x = 0; x < FINAL_GRID_W; x++)
    {
        for (y = 0; y < FINAL_GRID_H; y++)
        {
            Final_Blocked[x][y] = 0U;
            Final_Preferred[x][y] = 0U;
        }
    }

    route_x[0] = current_x;
    route_y[0] = current_y;
    return_x[0] = current_x;
    return_y[0] = current_y;

#if NODE_DEBUG
    printf("\r\n== A车 FinalProgram_A: %s team ==\r\n", FINAL_TEAM_NAME);
    printf("Final: start=(%u,%u) goal=(%u,%u), outbound + delivery + return\r\n",
           (unsigned int)FINAL_START_X, (unsigned int)FINAL_START_Y,
           (unsigned int)FINAL_GOAL_X, (unsigned int)FINAL_GOAL_Y);
    printf("Final: return recheck=%lums, dynamic BFS\r\n",
           (unsigned long)FINAL_RETURN_RECHECK_MS);
#endif

    delay_ms(NODE_START_DELAY_MS);

    /* ---------------------------- 去程 ---------------------------- */
    while ((current_x != FINAL_GOAL_X)
        || (current_y != FINAL_GOAL_Y))
    {
        if (outbound_steps >= FINAL_MAX_STEPS)
        {
#if NODE_DEBUG
            printf("Final: OUTBOUND STEP LIMIT REACHED\r\n");
#endif
            break;
        }

        Final_ReadIRSides(current_x, current_y,
                          &front, &back, &left, &right);
        Final_UpdateBlockedMap(current_x, current_y,
                               front, back, left, right);

        if (!Final_BfsNext(current_x, current_y,
                           FINAL_GOAL_X, FINAL_GOAL_Y,
                           &next_x, &next_y))
        {
#if NODE_DEBUG
            printf("Final: NO OUTBOUND PATH FROM (%u,%u), try one-step retreat\r\n",
                   (unsigned int)current_x, (unsigned int)current_y);
#endif
            if (Final_StepBackHistory(route_x, route_y, &route_len,
                                      &current_x, &current_y))
            {
                delay_ms(NODE_INERTIA_SETTLE_MS);
                continue;
            }

            Final_ClearBlockedMap();
            current_x = FINAL_START_X;
            current_y = FINAL_START_Y;
            route_x[0] = current_x;
            route_y[0] = current_y;
            route_len = 1U;
            outbound_steps = 0U;
            delay_ms(NODE_INERTIA_SETTLE_MS);
            continue;
        }

        mode = Final_ModeForStep(current_x, current_y, next_x, next_y);
        if (mode == 0xFFU) break;

#if NODE_DEBUG
        printf("Final: OUT %u (%u,%u) -> (%u,%u) %s\r\n",
               (unsigned int)(outbound_steps + 1U),
               (unsigned int)current_x, (unsigned int)current_y,
               (unsigned int)next_x, (unsigned int)next_y,
               Node_ModeName(mode));
#endif

        travel_result = Node_Travel(mode, current_x, current_y);
        if (travel_result == 2U)
        {
            Final_MarkBlockedForMode(current_x, current_y, mode);
            Node_Stop();
            delay_ms(NODE_INERTIA_SETTLE_MS);
            continue;
        }
        if (travel_result == 0U)
        {
            Node_Stop();
            break;
        }

        Node_Stop();
        delay_ms(NODE_INERTIA_SETTLE_MS);

        current_x = next_x;
        current_y = next_y;
        outbound_steps++;

        if (route_len >= FINAL_ROUTE_CAPACITY) break;
        route_x[route_len] = current_x;
        route_y[route_len] = current_y;
        route_len++;

#if NODE_DEBUG
        printf("Final: ARRIVED (%u,%u)\r\n",
               (unsigned int)current_x, (unsigned int)current_y);
#endif
    }

    if ((current_x == FINAL_GOAL_X) && (current_y == FINAL_GOAL_Y)
        && (route_len <= FINAL_ROUTE_CAPACITY))
    {
        uint8_t i;

        for (i = 0; i < route_len; i++)
            Final_Preferred[route_x[i]][route_y[i]] = 1U;

        Final_PrintPath("OUTBOUND", route_x, route_y, route_len);

#if NODE_DEBUG
        printf("Final: GOAL REACHED in %u steps, route nodes=%u\r\n",
               (unsigned int)outbound_steps, (unsigned int)route_len);
#endif

        if (Final_DeliveryTask())
        {
            /* ---------------------------- 返程 ---------------------------- */
            while ((current_x != FINAL_START_X)
                || (current_y != FINAL_START_Y))
            {
                if (return_steps >= FINAL_MAX_STEPS)
                {
#if NODE_DEBUG
                    printf("Final: RETURN STEP LIMIT REACHED\r\n");
#endif
                    break;
                }

                Final_ReadIRSides(current_x, current_y,
                                  &front, &back, &left, &right);

                /* Previously known obstacles skip the 2000 ms wait. */
                if (Final_IrHasUnknownObstacle(current_x, current_y,
                                               front, back, left, right))
                {
#if NODE_DEBUG
                    printf("Final: RETURN NODE OBSTACLE at (%u,%u), wait=2000ms\r\n",
                           (unsigned int)current_x, (unsigned int)current_y);
#endif
                    delay_ms(FINAL_RETURN_RECHECK_MS);
                    Final_ReadIRSides(current_x, current_y,
                                      &front, &back, &left, &right);
                    Final_UpdateBlockedMap(current_x, current_y,
                                           front, back, left, right);
                }

                if (!Final_BfsNext(current_x, current_y,
                                   FINAL_START_X, FINAL_START_Y,
                                   &next_x, &next_y))
                {
#if NODE_DEBUG
                    printf("Final: NO RETURN PATH FROM (%u,%u), try one-step retreat\r\n",
                           (unsigned int)current_x,
                           (unsigned int)current_y);
#endif
                    if (Final_StepBackHistory(return_x, return_y, &return_len,
                                              &current_x, &current_y))
                    {
                        delay_ms(NODE_INERTIA_SETTLE_MS);
                        continue;
                    }

                    Final_ClearBlockedMap();
                    current_x = FINAL_GOAL_X;
                    current_y = FINAL_GOAL_Y;
                    return_x[0] = current_x;
                    return_y[0] = current_y;
                    return_len = 1U;
                    return_steps = 0U;
                    delay_ms(NODE_INERTIA_SETTLE_MS);
                    continue;
                }

                mode = Final_ModeForStep(current_x, current_y,
                                         next_x, next_y);
                if (mode == 0xFFU) break;

#if NODE_DEBUG
                printf("Final: RET %u (%u,%u) -> (%u,%u) %s\r\n",
                       (unsigned int)(return_steps + 1U),
                       (unsigned int)current_x, (unsigned int)current_y,
                       (unsigned int)next_x, (unsigned int)next_y,
                       Node_ModeName(mode));
#endif

                travel_result = Node_Travel(mode, current_x, current_y);
                if (travel_result == 2U)
                {
                    if (Final_ReturnObstaclePersists(current_x,
                                                     current_y, mode))
                    {
                        Final_MarkBlockedForMode(current_x, current_y, mode);
                    }
                    Node_Stop();
                    delay_ms(NODE_INERTIA_SETTLE_MS);
                    continue;
                }
                if (travel_result == 0U)
                {
                    Node_Stop();
                    break;
                }

                Node_Stop();
                delay_ms(NODE_INERTIA_SETTLE_MS);

                current_x = next_x;
                current_y = next_y;
                return_steps++;

                if (return_len >= FINAL_ROUTE_CAPACITY) break;
                return_x[return_len] = current_x;
                return_y[return_len] = current_y;
                return_len++;

#if NODE_DEBUG
                printf("Final: RETURN ARRIVED (%u,%u)\r\n",
                       (unsigned int)current_x,
                       (unsigned int)current_y);
#endif
            }

            if ((current_x == FINAL_START_X)
                && (current_y == FINAL_START_Y))
            {
                success = 1U;
                Final_PrintPath("RETURN", return_x, return_y, return_len);
#if NODE_DEBUG
                printf("Final: RETURNED TO START, return steps=%u\r\n",
                       (unsigned int)return_steps);
#endif
            }
        }
    }

    Node_Stop();

    for (;;)
    {
        Node_Lamp(success ? 1U : 2U, tick);
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
