/* ============================================================================
 *  Align v2 —— 把车锁死在两条黑线的交叉点上
 *
 *  【功能】四个灰阶同时对准：前、后两块看到纵向黑线，左、右两块看到横向黑线，
 *          每一块都要求"黑线压在探头正中间"。横向平移、纵向平移、原地旋转
 *          三个自由度一起闭环，一直检测、一直修正 —— 对准之后不是停机不管，
 *          而是保持在死区里；被推歪了、推偏了，它自己会挪回来。
 *          放得太斜、四个灰阶都看不到线时，它会慢慢转着找（见"搜索"一节）。
 *
 *  【用法】把车放在两条黑线的交叉点上（车头顺着其中一条黑线最好，斜一点也行，
 *          斜太多它会自己转着找）。上电或按复位，车会挪到交叉点正中并摆正，
 *          然后停在那儿保持。
 *
 *  【当前档位】ALIGN_MODE = 2，且 ALIGN_SKIP_SETUP = 1 —— **上电直接直走巡线**：
 *          不做原地旋转对准。车由人摆到大致顺线的方向（车头顺着黑线即可，
 *          歪个十来度也行，阶段 B 会自己拧回来），上电就走。
 *          巡线**只看前灰阶**：偏了就朝线的方向【转向】拧回来，掉线就停住等。
 *          阶段 A（原地转对准）还在代码里，只是默认跳过；想单独调它就把
 *          ALIGN_SKIP_SETUP 设回 0。
 *          ALIGN_MODE = 0 只观察，= 1 只旋转对准（前+后）。
 *
 *  【用到的硬件】
 *      前灰阶  GW 8 路灰度   软 I2C  PB13 = SCL, PB14 = SDA, 地址 0x4F
 *      后灰阶  GW 8 路灰度   软 I2C  PD1  = SCL, PD2  = SDA, 地址 0x4F
 *      左灰阶  GW 8 路灰度   软 I2C  PC8  = SCL, PC9  = SDA, 地址 0x4F
 *      右灰阶  GW 8 路灰度   软 I2C  PC3  = SCL, PC2  = SDA, 地址 0x4F
 *      四轮驱动板 0x34       软 I2C  PF6  = SCL, PF5  = SDA
 *      绿灯 PA7 / 红灯 PA6
 *      串口 USART1, PA9 = TX, 19200 8N1（可以不接）
 *      其它外设一律不碰。
 *
 *  【原理】车体系：x 朝前、y 朝左。前灰阶在 x = +a 处、8 个探头沿 y 排开；
 *          后灰阶在 x = -a；左灰阶在 y = +b、探头沿 x 排开；右灰阶在 y = -b。
 *          记 y_c 为车心相对纵向黑线的横向偏移、x_c 为相对横向黑线的纵向偏移、
 *          theta 为车头与黑线的夹角，四条读数分别是
 *              前 y_f ≈ y_c - a*theta          后 y_b ≈ y_c + a*theta
 *              左 x_L = b*tan(theta) - x_c/cos(theta)
 *              右 x_R = -b*tan(theta) - x_c/cos(theta)
 *          于是（E_* 是各灰阶的带符号位置读数）
 *              横向平移误差 ∝ (E_F + E_B)      <- 加：theta 抵消
 *              转角误差     ∝ (E_B - E_F)      <- 减：y_c 抵消（左/右同理）
 *              纵向平移误差 ∝ (E_L + E_R)
 *          注意这三个式子跟轮子那三个正交组合是同一种结构：平移是共模、旋转是
 *          差模。所以三个通道一阶上互不干扰，而且"该平移平移、该旋转旋转"不是
 *          两套模式切换，而是每一帧同时算、直接相加。
 *          对齐的位姿是自洽的：车心落在交叉点且轴对准时，纵向线正好穿过前、后
 *          灰阶的中点，横向线正好穿过左、右灰阶的中点 —— 四个同时"线在中间"。
 *          a、b、探头间距都不需要知道，只影响增益比例，而三个增益本来就各自调。
 *
 *  【判据为什么与模块极性无关】
 *          模块数字量到底是"1 代表黑"还是"1 代表白"，整个仓库查不到
 *          （GREY_driver.h 没说，驱动注释没说，move.c 的 ~raw & 0x18 预设了一种）。
 *          位置读数用反对称权重（探头 1..8 = +6,+4,+2,+1,-1,-2,-4,-6）加权求和，
 *          总和为 0，所以极性只会让结果整体变号 —— 而整体变号跟三个输出符号
 *          是同一件事，会被 ALIGN_SIGN_LAT/YAW/FWD 吸收掉，不需要专门的极性宏。
 *          有效性判据也只用"1 的个数"和"变化沿个数"，同样跟极性无关：
 *              全 0 / 全 1        -> 整块一个颜色，没有线（或是读回来的垃圾）
 *              1/0 交界超过 2 处  -> 不是一整段连续的黑，是噪声
 *          剩下的都算"有线"，正好也容得下黑线比较宽、居中时压住 4 个探头的情况。
 *
 *  【捕捉范围】四个灰阶装在车身四条边中点，离车心约 114mm，探头半跨约 25~28mm。
 *          于是"看得见线"的前提是线还落在探头跨距以内：
 *              车头夹角   约 ±13 度   （114 * tan13° ≈ 26mm）
 *              横向偏移   约 ±28mm    （再大前/后就丢了纵向线）
 *              纵向偏移   约 ±28mm    （再大左/右就丢了横向线）
 *          角度这一条前/后和左/右是**同时**失效的（都由车头夹角决定），所以放太
 *          歪就是四个一起瞎，程序只能慢慢原地转着找，转进 ±13 度就接手。
 *          旧版只用一块前灰阶，所以能容忍 ±90 度；要同时对准四个就容不下了。
 *          线性那两条不一样：横向超限时左/右还看得见，但横向通道没数据可用，
 *          这时程序**停住亮红灯等**，不会瞎开电机 —— 原地转救不了纯横向偏移，
 *          所以这里没做自动搜索，宁可停着也不要乱跑。
 *
 *  【上车标定】见同目录 对准测试说明.md。先烧 ALIGN_OBSERVE_ONLY = 1：
 *          只打印、不驱动，用手推车看四个读数怎么变，把两个 ALIGN_SIGN_B/R 和
 *          三个 ALIGN_SIGN_* 定下来，再改回 0。
 * ========================================================================== */

#include "main.h"

/* ---------------------------------------------------------------------------
 *  一、可调参数
 * ------------------------------------------------------------------------- */

/* 跑哪个模式。改这个数，重新编译烧录。

      0 = 只观察：一个电机指令都不发，只按 5Hz 打印四路原始字节、四路位置读数
          和三个通道值。用来标定下面那几个符号，也顺便看清模块极性。
      1 = 只旋转对准：只开 w 一个通道，车只转不挪。歪了自己转到摆正就停。
          误差用前/后那一对（ALIGN_YAW_FROM_LR = 1 就改用左/右）。
      2 = 直走巡线（默认跳过了启动对准）：
          阶段 A —— 原地转，让"前灰阶"和"右灰阶"的黑线同时落到探头中间。
                    只转不挪，所以要求起步时车心大致压在两条线的交叉点上。
                    **默认不做**（ALIGN_SKIP_SETUP = 1）：角上绕角点转不可观测，
                    这块本来也靠不住，车头交给阶段 B 自己拧。
          阶段 B —— 以 ALIGN_CRUISE_SPEED 直走，**循迹只看前灰阶**：
                    偏了就朝线的方向【转向】拧回来。不用平移 ——
                    一块探头只量得到"线偏了多少"、量不到"车头歪了多少"，
                    平移会让车以任何角度横着蹭线走（看着就是斜着走）。
                    角上只有前方和右方两条臂，车尾那块灰阶下面根本没有线，
                    能用来循迹的只有前面这一块。
                    前灰阶**掉线了不会立刻停**：先按最后那次的纠偏方向继续走
                    ALIGN_LOST_MAX 个周期（约 2~3 秒），还找不回来才停住等着。
                    不原地打转。
                    走到**十字/丁字路口**、前灰阶整个压在那条横线上时**不转向**，
                    直着开过去（判据见 ALIGN_BLACK_LEVEL 那段和 Strip_Blob）。
          调试开关（都在下面）：
              ALIGN_SKIP_SETUP = 1  跳过阶段 A，上电直接进阶段 B（**默认**）
              ALIGN_B_FIX      = 0  阶段 B 一个传感器都不看，四个轮子给同一个速度
                                    （单独验"底盘直走直不直"，跟 MoveTest 第一步
                                     发的指令一模一样） */
#define ALIGN_MODE          2

#define ALIGN_OBSERVE_ONLY  (ALIGN_MODE == 0)

/* 串口打印开关。打印是阻塞的，19200 波特下一行约 90 字节约 50ms，
   所以只在每 ALIGN_PRINT_EVERY 个周期打一行，免得拖慢控制周期。 */
#define ALIGN_DEBUG         1
#define ALIGN_PRINT_EVERY   250

/* VOFA+ 上位机波形（JustFloat 协议，走 USART1）。调增益的时候看波形比看数字
   快得多。默认关；打开后每 ALIGN_VOFA_EVERY 个周期发一帧 7 个 float。 */
#define ALIGN_VOFA          0
#define ALIGN_VOFA_EVERY    10

/* 控制周期（ms）。采样四路灰阶 + 下发一帧电机指令的实际耗时还要加上，
   所以真实周期比这个数略大（大约 250~300Hz）。设 0 就是全速跑。 */
#define ALIGN_TICK_MS       4

/* 死区（权重单位，单路读数 E 的量程是 ±13，靠中心每过一个探头变 3 个单位）。
   误差绝对值小于它就当 0，避免探头量化噪声让四个轮子一直嗡嗡抖。
   嫌对得不够准就调小，嫌到位后抖就调大。 */
#define ALIGN_DEADBAND      4

/* 非零输出的最小幅值，用来克服静摩擦：算出来只有 1、2 的话，板子给了转速
   轮子却转不动，车会一动不动地"卡"在那儿，看起来像程序没反应。
   原工程巡线用的是 10~15，这里比它小是因为对准只需要很慢地挪。 */
#define ALIGN_MIN_CMD       6

/* 单通道限幅。三个通道叠加后单个轮子最大是 3 倍，也就是 36，
   远小于 motor.c 里 MOTOR_Data 的 int8 范围（±127），不会溢出。 */
#define ALIGN_MAX_CMD       12

/* 三个通道的定点增益（实际增益 = 这个数 / 8），默认都是 1.0。
   发散/画圈就三个一起减半，收敛太慢就加大。 */
#define ALIGN_K_LAT         8
#define ALIGN_K_YAW         8
#define ALIGN_K_FWD         8

/* 通道一阶低通强度（右移位数）。0 = 不滤波。
   设成 1 相当于每周期只走一半，既是低通也是斜率限制，能压住瞬态的画圈。 */
#define ALIGN_FILT_SHIFT    0

/* 一对灰阶都凑不齐连续多少个周期，就认为"看不见线了"，开始慢慢转着找。
   25 个周期约 100ms：短时间的掉线先保持不动，别一惊一乍地转。 */
#define ALIGN_ACQ_AFTER     25

/* 搜索时的旋转速度和方向。速度沿用原工程 Revolve() 的量级；方向随意，
   反正往哪边转都能在 180 度之内转进 ±15 度的窗口。 */
#define ALIGN_SPIN_SPEED    10
#define ALIGN_SPIN_DIR      (+1)

/* ---- 十字/丁字路口：压在横线上时不转向 ----------------------------------
 *
 * 前灰阶正常跟的是一条**纵向**黑线，横着排在探头底下，只有 1~3 个探头是黑的。
 * 走到十字或丁字路口时，横向那条线是和探头排**平行**的，于是探头上会同时压住
 * 纵向那条（中间 4、5 号）和横向那一整条（一整侧全是黑的，比如 1~5 号全黑）。
 *
 * 这时候算出来的 E 是被那一大块黑拉偏的，**那不是"线偏了"，不能拿它转向** ——
 * 照它转的话车会在路口上莫名其妙地拐一下。判据是"形状"：
 *
 *      探头 4 和 5 号都是黑，**并且**（1~4 号全黑 **或者** 5~8 号全黑）
 *          -> 认为压在横向线上了，纠偏量清零，直着开过去
 *
 * 只在**模式 2 阶段 B**用（横向线是路口，正常路面不会有）。
 *
 * !! ALIGN_BLACK_LEVEL —— 模块的数字量里"黑"是 1 还是 0，仓库里查不到
 *    （文件顶上有说明），这里只能先按一个值写。**看一眼串口那行 `F` 后面的
 *    十六进制就能定**：
 *        车压在黑线上时，字节里 1 比 0 多  ->  黑 = 0（默认；原工程
 *                                            `~raw & 0x18` 也是这么假设的）
 *        字节里 0 比 1 多                  ->  黑 = 1
 *    设反了的症状很好认：判断反了之后，它会在**车偏到一边、另一侧全白**的时候
 *    才触发 "不转向" —— 也就是**最该转向的时候它偏偏不转**，车会一路冲出线外。
 *    真反了就改这个数。
 * ------------------------------------------------------------------------ */
#define ALIGN_BLACK_LEVEL   0

/* ---------------------------------------------------------------------------
 *  二、符号标定（硬件装法决定，代码里查不到）
 *
 *  前、后两块灰阶是同一套支架装的，但有可能装反了 180 度；左、右同理。
 *  装反了的话 (E_F + E_B) 就不是"横向平移"而是"转角"，这种错**没法**用输出
 *  符号补救，必须在这里翻。所以这里只需要两个"配对"符号：
 *
 *      ALIGN_SIGN_B：把 E_B 相对 E_F 取反
 *      ALIGN_SIGN_R：把 E_R 相对 E_L 取反
 *
 *  为什么不需要给 F 和 L 也各来一个？因为同时翻一对是"整体变号"，那跟三个输出
 *  符号是同一件事，重复了。真正独立的只有这两个"配对"符号。
 *  模块极性也不用管：它对四个灰阶一视同仁，整体变号同样被输出符号吸收。
 *
 *  怎么定（ALIGN_OBSERVE_ONLY = 1，打印里看四个 E）：
 *
 *     把车整体往左推约 1cm（车头别歪）：
 *         前、后两个 E 应该"同增"或"同减"  -> 对；一增一减就翻 ALIGN_SIGN_B
 *     把车整体往前推约 1cm：
 *         左、右两个 E 应该"同增"或"同减"  -> 对；一增一减就翻 ALIGN_SIGN_R
 *     原地转 10 度（复核）：
 *         前、后应该"一增一减"，左、右也应该"一增一减"
 *
 *  道理：平移时 y_f = y_c - a·theta 和 y_b = y_c + a·theta 里的 y_c 一起变，
 *  两块同向；转的时候动的是 a·theta 那一项，两块一正一负，所以反向。
 *  判据是"同向/反向"而不是"变大/变小"，所以不用先知道模块极性。
 * ------------------------------------------------------------------------- */
#define ALIGN_SIGN_B        (+1)
#define ALIGN_SIGN_R        (+1)

/* ---------------------------------------------------------------------------
 *  三、三个输出通道的整体符号
 *
 *  上面那两个是"配对"符号，这两个是"通道"符号：把整个通道取反。
 *  如果车朝着"离开黑线"的方向跑、越修越偏，就是对应的那个反了，翻面。
 *  只影响正负，不影响幅度。用哪个通道反了就翻哪个（观察模式下按单轴试）。
 * ------------------------------------------------------------------------- */
#define ALIGN_SIGN_LAT      (+1)    /* 横向平移 */
#define ALIGN_SIGN_YAW      (-1)    /* 原地旋转。2026-10-10 实测：+1 时车往左边越走越偏（正反馈），翻成 -1 */
#define ALIGN_SIGN_FWD      (+1)    /* 纵向平移 */

/* 转角通道用哪一对算：0 = 用前/后 (E_B - E_F)，1 = 用左/右 (E_L - E_R)。
   两对都能量转角；正常用 0。注意这两对的相对符号没标定过，所以**不要**把
   两个平均起来，选一对用就行。 */
#define ALIGN_YAW_FROM_LR   0

/* 模式 1 = 只开"原地旋转"一个通道，横向和纵向都不发，车只转不挪。
   用来单独验证旋转这一路（顺便把 ALIGN_SIGN_YAW 标定掉）：
   一次只试一个自由度，转不动、转反了、来回摆，一眼就能对上是哪一步错了。
   注意模式 1 只清零"输出"，三个通道的误差值照样算、照样串口打印，
   所以这个模式下也顺手把四块灰阶的读数看了。 */
#define ALIGN_ONLY_YAW      (ALIGN_MODE == 1)

/* ---------------------------------------------------------------------------
 *  三之二、模式 2（启动对准 + 直走巡线）的参数
 * ------------------------------------------------------------------------- */

/* 阶段 B 直走的速度。四个轮子给同一个速度时，这一帧跟 MoveTest 第一步"前进"
   发的指令是一模一样的 —— MoveTest 那边是 MOVETEST_SPEED = 10。
   2026-10-10：原来 12 太慢，提到 20。嫌快就减，嫌慢就加。 */
#define ALIGN_CRUISE_SPEED  20

/* 阶段 A 旋转对准的通道增益（实际增益 = 这个数 / 8）。 */
#define ALIGN_K_SETUP       8

/* 阶段 A 判定"摆正了"的容差（权重单位；靠中心每过一个探头约 3 个单位）。
   判的是 (E_F + E_R) 这个**和**（跟控制律调的是同一个量，见下面的注释），
   落进 ±这个数就切到阶段 B。
   **必须 >= ALIGN_DEADBAND**：比死区小的话这个条件永远不成立，车会一直卡在阶段 A。 */
#define ALIGN_SETUP_BAND    4

/* 阶段 A 旋转的方向。越转前/右越偏（冲过头发散）就改成 -1。 */
#define ALIGN_SIGN_SETUP    (+1)

/* 阶段 B 的纠偏方式。把它设成 0 就完全不纠偏 —— 四个轮子给同一个速度，
   一个传感器都不掺。**先把这个设成 0 验一次"底盘直走直不直"**，
   确认底盘没问题再打开纠偏。 */
#define ALIGN_B_FIX         1       /* 0 = 不纠偏（只直走） 1 = 转向纠偏  2 = 平移纠偏 */

/* 上电后跳过阶段 A，直接进阶段 B。
   2026-10-10 起默认就是 **1**：车由人摆到大致顺线的方向，上电直接直走巡线，
   不做原地旋转对准（角上绕角点转不可观测，阶段 A 本来也靠不住，见
   align-mode2-corner-start 那条笔记）。想单独调阶段 A 再设回 0。 */
#define ALIGN_SKIP_SETUP    1

/* ---- 阶段 B 的纠偏通道参数 ------------------------------------------------
 *
 * 这三个数跟阶段 A 的那套（ALIGN_DEADBAND / ALIGN_MIN_CMD / ALIGN_MAX_CMD）
 * **必须分开**，不能共用。原因是"最小起步量"：
 *
 *   阶段 A 是原地转，四个轮子本来就从 0 起步，给 1、2 的转速轮子克服不了静摩擦，
 *   所以那边需要 ALIGN_MIN_CMD 抬一手。
 *   阶段 B 车**本来就在往前走**（四个轮子都有 ALIGN_CRUISE_SPEED），根本没有
 *   "起不来"这回事。这时候再套一个最小起步量，纠偏量就变成了"非 0 就是 6"的
 *   开关控制；再加上原来限幅 12 > 直走速度 10 —— 纠偏量一饱和就把一侧两个轮子
 *   算成了 10-12 = -2（-2 转不动），**车就只剩另外一侧两个轮子在转，原地打转**。
 *   这就是"只有右边的轮子会转"的来源。
 *
 * 所以这里：没有最小起步量；限幅必须**严格小于**直走速度，保证任何误差下四个
 * 轮子都还是正的、都在往前走。下面那个 #error 就是防这个的。
 * ------------------------------------------------------------------------- */
#define ALIGN_B_DEADBAND    2       /* |E_F| 小于它就当 0（约 ±5mm，防止探头量化噪声抖）。
                                       **这个数直接决定"响应快不快"**：探头每过一个
                                       探头位置读数才变约 3 个单位，死区 3 就等于"偏了
                                       一整个探头都不理"，看着就是反应慢。 */
#define ALIGN_B_K           8       /* 增益：纠偏量 = E_F × 这个数 ÷ 8。8 就是"纠偏量 = E_F"。
                                       **响应慢就加这个**（4 → 8 → 12），太猛画龙就减。 */
#define ALIGN_B_MAX         5       /* 纠偏限幅，**必须 < ALIGN_CRUISE_SPEED**。
                                       只管大偏差的封顶，不影响小偏差的响应速度。 */

#if (ALIGN_B_MAX >= ALIGN_CRUISE_SPEED)
#error "ALIGN_B_MAX must be smaller than ALIGN_CRUISE_SPEED"
#endif

/* 阶段 B 丢线之后**不立刻停**，按最后那次的纠偏方向继续走这么多周期
   （一个周期约 4~6ms，所以 500 大约是 2~3 秒），还找不回来才停住。
   原来是一丢线就停车，结果"传感器一没数据车就不走"——
   前灰阶能看见的横向范围只有 ±28mm 左右，轻轻一偏就丢线了。
   丢线时保持最后一次的转向，多数情况是"偏出去一点"，接着这么拧一两个身位
   线就回来了。**必须 >= 1**；设 1 就是丢线后下一帧立刻停。 */
#define ALIGN_LOST_MAX      500

/* LED 是低有效（PA6/PA7 输出 0 点亮，LED_Init 之后两个都是灭的）。
   实测亮灭跟说明相反就把这两个数对调。 */
#define ALIGN_LED_ON        0
#define ALIGN_LED_OFF       1

/* ---------------------------------------------------------------------------
 *  四、驱动里没有在头文件里声明的底层函数，这里自己补一下
 * ------------------------------------------------------------------------- */
extern sw_i2c_interface_t f_i2c_interface;
extern sw_i2c_interface_t b_i2c_interface;
extern sw_i2c_interface_t l_i2c_interface;
extern sw_i2c_interface_t r_i2c_interface;

extern int8_t f_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t b_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t l_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);
extern int8_t r_sw_i2c_mem_read(sw_i2c_interface_t *, uint8_t, uint8_t, uint8_t *, uint8_t);

/* 直接用带返回值的 read_byte 读开关量，而不是调 x_GW_digital_Read()。
   后者把 f_sw_i2c_read_byte 的返回值丢掉了，I2C 一失败就把**没初始化的局部
   变量**返回出来 —— 那是个看着合法的垃圾值，拿它开电机会出事。 */
extern int8_t f_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);
extern int8_t b_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);
extern int8_t l_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);
extern int8_t r_sw_i2c_read_byte(sw_i2c_interface_t *, uint8_t, uint8_t *);

/* 0x4F 是 7 位地址，软 I2C 收发用的是 8 位形式，要左移一位。 */
#define ALIGN_GW_ADDR8      ((uint8_t)(GW_GRAY_ADDR_DEF << 1))

/* 位序翻转。四块灰阶驱动（F/B/L/R_GREY_driver.c）末尾的 x_GW_digital_eight()
   都调用了 reverse()，而它的定义在 APP\trace\trace.c —— 那是比赛固件才编的
   文件，本工程文件清单里没有它，链接会报：
       Error: L6218E: Undefined symbol reverse (referred from b_grey_driver.o)
   这里补一个同名同实现的函数（照抄 trace.c 的写法），纯粹为了满足链接，
   本程序自己没有用到它。Test_Grey\grey_test_main.c 也是同样的绕法。
   别当死代码删掉。 */
uint8_t reverse(uint8_t x)
{
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));

    return (uint8_t)((x >> 4) | (x << 4));
}

/* ---------------------------------------------------------------------------
 *  五、灰阶：读一块、判有效性、算带符号位置
 * ------------------------------------------------------------------------- */

typedef struct {
    void                (*init)(void);
    int8_t              (*read_byte)(sw_i2c_interface_t *, uint8_t, uint8_t *);
    sw_i2c_interface_t  *iface;
    int8_t              (*mem_read)(sw_i2c_interface_t *, uint8_t,
                                    uint8_t, uint8_t *, uint8_t);
    int8_t               dir;
    const char          *name;
} Align_Strip_t;

/* 顺序是前、后、左、右，下面处处按这个下标来。 */
#define ALIGN_F 0
#define ALIGN_B 1
#define ALIGN_L 2
#define ALIGN_R 3

/* 只有 B 和 R 带符号，见上面"符号标定"那一节。 */
static const Align_Strip_t STRIP[4] = {
    { F_GW_digital_Init, f_sw_i2c_read_byte, &f_i2c_interface, f_sw_i2c_mem_read,
      (+1), "F" },
    { B_GW_digital_Init, b_sw_i2c_read_byte, &b_i2c_interface, b_sw_i2c_mem_read,
      (int8_t)ALIGN_SIGN_B, "B" },
    { L_GW_digital_Init, l_sw_i2c_read_byte, &l_i2c_interface, l_sw_i2c_mem_read,
      (+1), "L" },
    { R_GW_digital_Init, r_sw_i2c_read_byte, &r_i2c_interface, r_sw_i2c_mem_read,
      (int8_t)ALIGN_SIGN_R, "R" },
};

/* 原始字节里有多少个 1。 */
static uint8_t Strip_Popcount(uint8_t x)
{
    uint8_t n = 0;

    while (x) {
        n = (uint8_t)(n + (x & 0x01u));
        x = (uint8_t)(x >> 1);
    }
    return n;
}

/* 相邻两位不同的次数。一整段连续的黑线只会产生 1~2 次变化。 */
static uint8_t Strip_Trans(uint8_t x)
{
    uint8_t t = 0;
    uint8_t i;

    for (i = 1; i < 8; i++) {
        if (((x >> i) & 0x01u) != ((x >> (i - 1)) & 0x01u)) t++;
    }
    return t;
}

/* 这块灰阶是不是"有一条线压在探头上"。与模块极性无关，理由见文件头。
   全 0 / 全 1 = 整块一个颜色（没有线，或者读到的是垃圾）；交界超过 2 处 =
   不是一整段连续的黑，也是垃圾。剩下都算有线 —— 这样黑线偏宽、居中时压住 4 个
   探头的情况也照样成立。 */
static uint8_t Strip_Valid(uint8_t raw)
{
    uint8_t pc = Strip_Popcount(raw);

    if ((pc == 0) || (pc == 8)) return 0;
    if (Strip_Trans(raw) > 2) return 0;

    return 1;
}

/* 黑线落在哪一头、离探头中间多远，带符号。
       bit0..bit7 = 探头 1..8，权重 +6,+4,+2,+1,-1,-2,-4,-6
   这跟 GREY_driver.h 里 {F,B,L,R}_GW_digital_eight() 是**逐位相同**的结果
   （256 个输入全验过）：那两个函数内部先 reverse(~raw) 再按同一组权重求和，
   因为权重反对称（w[i] = -w[7-i]，总和为 0），reverse 和取反恰好抵消。
   直接对原始字节加权是一模一样的，还能省掉一次 I2C 读。
   返回值范围是 ±13（最大子集和 6+4+2+1），靠中心每过一个探头约变 3 个单位：
   0 = 线在正中间，正数表示线偏向探头 1 那一头。 */
static int16_t Strip_Pos(uint8_t raw)
{
    static const int8_t W[8] = { 6, 4, 2, 1, -1, -2, -4, -6 };
    int16_t s = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (raw & (uint8_t)(1u << i)) s = (int16_t)(s + W[i]);
    }
    return s;
}

/* 是不是"压在横向线（十字/丁字路口）上了"。判据和理由见文件上方
   ALIGN_BLACK_LEVEL 那段。注意它**不依赖** Strip_Valid —— 压住一整侧的时候
   交界只有 1~2 处，Strip_Valid 照样判"有线"。 */
static uint8_t Strip_Blob(uint8_t raw)
{
    uint8_t b  = (uint8_t)ALIGN_BLACK_LEVEL;
    uint8_t lo = (uint8_t)(b ? 0x0Fu : 0x00u);   /* 1~4 号（bit0..bit3）全黑时低半字节的值 */
    uint8_t hi = (uint8_t)(b ? 0xF0u : 0x00u);   /* 5~8 号（bit4..bit7）全黑时高半字节的值 */

    /* 探头 4 = bit3，探头 5 = bit4 */
    if ((uint8_t)((raw >> 3) & 1u) != b) return 0;
    if ((uint8_t)((raw >> 4) & 1u) != b) return 0;

    if ((uint8_t)(raw & 0x0Fu) == lo) return 1;  /* 左侧 1~4 号全黑 */
    if ((uint8_t)(raw & 0xF0u) == hi) return 1;  /* 右侧 5~8 号全黑 */

    return 0;
}

/* 灰阶自检：PING 一下，模块回 0x66 才算通。
   软 I2C 初始化时会多打一个 start 把总线掐住，所以第一次读一定失败，先丢一次。 */
static uint8_t Strip_Ping(const Align_Strip_t *s)
{
    uint8_t rsp = 0;

    s->mem_read(s->iface, ALIGN_GW_ADDR8, GW_GRAY_PING, &rsp, 1);   /* 丢掉 */
    rsp = 0;
    if (s->mem_read(s->iface, ALIGN_GW_ADDR8, GW_GRAY_PING, &rsp, 1) != 0) {
        return 0;
    }
    return (uint8_t)(rsp == GW_GRAY_PING_OK);
}

/* ---------------------------------------------------------------------------
 *  六、电机
 * ------------------------------------------------------------------------- */

/* 把四个轮子的目标转速发给驱动板。
   通道顺序照抄原工程 trace.c / move.c 的写法，是 (203, 202, 204, 201)，
   跟 201~204 的顺序不一样；照抄就行，自己重排会让四个轮的对应关系出错。 */
static void Wheel_Output(int16_t m201, int16_t m202, int16_t m203, int16_t m204)
{
    Motor_201.PID_Output = (float)m201;
    Motor_202.PID_Output = (float)m202;
    Motor_203.PID_Output = (float)m203;
    Motor_204.PID_Output = (float)m204;

    MOTOR_CONTROL_DATA_Predit(&Motor_203, &Motor_202, &Motor_204, &Motor_201);
}

/* 三个通道叠加成一帧。本车的轮速空间正好是一组 Hadamard 基，四个组合两两正交，
   所以可以直接相加：

      前后平移 (+1,+1,+1,+1)     左右平移 (+1,-1,+1,-1)     原地旋转 (+1,+1,-1,-1)

   第四组 (+1,-1,-1,+1) 是四个轮子互相顶死的空转：对角上的轮子成对顶死，
   合力和合力矩都是 0，车不动。它不参与混合。

   —— 旋转那一组原来是按 (+1,-1,-1,+1) 写的，那是错的，正好落到了空转组上，
      所以不管给多大速度，车都原地不动（MoveTest 第 6 步实测：轮子转，车不动）。
      2026-10-10 用 MoveTest 点名测出：
          Motor_201 = 右前, 202 = 右后, 203 = 左后, 204 = 左前，
          四个电机"正转"都是车轮前进方向（四轮同向 = 车往前走），
          正负号驱动板是认的（正转反转真的相反）。
      由此得出：原地旋转 = 右侧两轮（201/202）一组、左侧两轮（203/204）一组，
      两组反向，即 (+1,+1,-1,-1)。至于哪边是左转，取决于轮子装的正反，
      用 ALIGN_SIGN_YAW / ALIGN_SPIN_DIR 现场调。 */

static int16_t Clamp(int16_t v, int16_t lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* 上一帧真正下发的四个轮速，顺序是 201/202/203/204，只给串口打印用。
   一旦"只有某几个轮子转"，看这一行就知道是指令算错了还是下层的问题：
   比如只看到右边的轮子转，这一行应该就是 20 20 -2 -2 之类的。 */
static int16_t g_m[4];

static void Wheel_Mix(int16_t vx, int16_t vy, int16_t w)
{
    int16_t lim  = (int16_t)(ALIGN_MAX_CMD * 3);    /* 三个通道叠加后最大到 3 倍 */
    int16_t m201 = Clamp((int16_t)(vx + vy + w), lim);
    int16_t m202 = Clamp((int16_t)(vx - vy + w), lim);
    int16_t m203 = Clamp((int16_t)(vx + vy - w), lim);
    int16_t m204 = Clamp((int16_t)(vx - vy - w), lim);

    g_m[0] = m201;  g_m[1] = m202;  g_m[2] = m203;  g_m[3] = m204;

    Wheel_Output(m201, m202, m203, m204);
}

/* 四个轮子全部停 */
static void Wheel_Stop(void)
{
    Wheel_Mix(0, 0, 0);
}

/* ---------------------------------------------------------------------------
 *  七、控制律与显示
 * ------------------------------------------------------------------------- */

/* 误差 -> 指令：死区、增益（除以 8）、限幅、最小起步量。
   最后那一步是必须的：算出来只有 1、2 的话，板子给了转速但轮子克服不了
   静摩擦，车会一动不动地"卡"在那儿，看起来像程序没反应。 */
static int16_t Align_Chan(int16_t u, int16_t k)
{
    int32_t v;

    if ((u > -ALIGN_DEADBAND) && (u < ALIGN_DEADBAND)) return 0;

    v = ((int32_t)u * (int32_t)k) / 8;

    if (v >  ALIGN_MAX_CMD) v =  ALIGN_MAX_CMD;
    if (v < -ALIGN_MAX_CMD) v = -ALIGN_MAX_CMD;

    if ((v > 0) && (v <  ALIGN_MIN_CMD)) v =  ALIGN_MIN_CMD;
    if ((v < 0) && (v > -ALIGN_MIN_CMD)) v = -ALIGN_MIN_CMD;

    return (int16_t)v;
}

/* 阶段 B 的纠偏通道：跟 Align_Chan 的区别是**没有最小起步量**（车本来就在走，
   不存在"转速太小克服不了静摩擦"），而且限幅比直走速度小，保证纠偏再大也不会
   把某一侧的轮子刹到 0。纠偏量 = u × ALIGN_B_K ÷ 8，默认就是"等于 u"。 */
static int16_t Align_BChan(int16_t u)
{
    int32_t v;

    if ((u > -ALIGN_B_DEADBAND) && (u < ALIGN_B_DEADBAND)) return 0;

    v = ((int32_t)u * (int32_t)ALIGN_B_K) / 8;

    if (v >  ALIGN_B_MAX) v =  ALIGN_B_MAX;
    if (v < -ALIGN_B_MAX) v = -ALIGN_B_MAX;

    return (int16_t)v;
}

/* 一阶低通，ALIGN_FILT_SHIFT = 0 时原样返回。 */
static int16_t Align_Filt(int16_t prev, int16_t x)
{
#if (ALIGN_FILT_SHIFT > 0)
    return (int16_t)(prev + ((x - prev) >> ALIGN_FILT_SHIFT));
#else
    (void)prev;
    return x;
#endif
}

/* 一个字节 -> 两位十六进制。缓冲区由调用者提供，不能共用一个 static 缓冲：
   printf 的实参求值顺序是未指定的，共用的话四个参数会都指向同一个缓冲，
   打印出来全是最后一个值。 */
static char *Hex2(char *o, uint8_t v)
{
    static const char D[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

    o[0] = D[(v >> 4) & 0x0F];
    o[1] = D[v & 0x0F];
    o[2] = '\0';

    return o;
}

/* ---------------------------------------------------------------------------
 *  八、主程序
 * ------------------------------------------------------------------------- */
int main(void)
{
    uint8_t  raw[4];
    int16_t  e[4];
    uint8_t  ok[4];
    uint8_t  i;
    uint8_t  pair_fb, pair_lr, all_four, pair_use;
#if (ALIGN_MODE == 2)
    uint8_t  pair_fr;               /* 模式 2 阶段 A 要的是"前 + 右"这一对 */
    uint8_t  setup_done = ALIGN_SKIP_SETUP;   /* 阶段 A 完成、已经切到阶段 B */
    int16_t  u_setup;               /* 阶段 A 的旋转误差 */
    uint16_t lost = 0;              /* 阶段 B 连续丢线的周期数（见 ALIGN_LOST_MAX） */
    int16_t  w_hold = 0;            /* 丢线期间保持的最后一次纠偏量 */
    uint8_t  blob = 0;              /* 前灰阶压在横向线上（十字/丁字路口），见 Strip_Blob */
#endif
    uint8_t  acq = 0;
    uint8_t  searching;
    int16_t  u_lat, u_yaw, u_fwd;
    int16_t  vx, vy, w;
    int16_t  f_lat = 0, f_yaw = 0, f_fwd = 0;
    uint32_t n = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);              /* delay_us / delay_ms 要用，必须最先做 */
    LED_Init();
    USART1_Init(19200);

    LED_RED   = ALIGN_LED_OFF;
    LED_GREEN = ALIGN_LED_OFF;

    /* 电机先初始化并把四个轮子按死为 0：万一哪次 I2C 写丢了，板子会一直保持
       上一次的转速，所以停下来之后也要周期性补发 0。 */
    MOTOR_Init();
    Wheel_Stop();

    for (i = 0; i < 4; i++) STRIP[i].init();

    delay_ms(300);                  /* 等灰阶模块上电稳定 */

#if ALIGN_DEBUG
    printf("\r\nAlign v2: hold at the crossing   observe=%d\r\n",
           (int)ALIGN_OBSERVE_ONLY);
#endif

    /* 四块灰阶都 PING 通之前不发任何电机指令。没通就红灯慢闪一直重试
       （重新 init 一遍顺带把卡住的总线放掉）。 */
    for (;;) {
        uint8_t bad = 0;

        for (i = 0; i < 4; i++) {
            if (!Strip_Ping(&STRIP[i])) {
                bad++;
#if ALIGN_DEBUG
                printf("Align: GREY %s no response (PING != 0x66)\r\n", STRIP[i].name);
#endif
            }
        }
        if (bad == 0) break;

        Wheel_Stop();
        LED_RED = !LED_RED;
        delay_ms(250);
        for (i = 0; i < 4; i++) STRIP[i].init();
        delay_ms(250);
    }

    LED_RED = ALIGN_LED_OFF;
#if ALIGN_DEBUG
    printf("Align: four grey OK, start\r\n");
#endif

    /* ---------------- 主循环：一直检测、一直修正，永不退出 ---------------- */
    for (;;) {
        /* --- 采样四块灰阶 --- */
        for (i = 0; i < 4; i++) {
            if (STRIP[i].read_byte(STRIP[i].iface, ALIGN_GW_ADDR8, &raw[i]) != 0) {
                ok[i] = 0;          /* I2C 失败：这一路的读数不可信，本周期丢掉 */
            } else {
                ok[i] = Strip_Valid(raw[i]);
            }

            e[i] = ok[i] ? (int16_t)(STRIP[i].dir * Strip_Pos(raw[i])) : 0;
        }

        pair_fb  = (uint8_t)(ok[ALIGN_F] && ok[ALIGN_B]);
        pair_lr  = (uint8_t)(ok[ALIGN_L] && ok[ALIGN_R]);
        all_four = (uint8_t)(pair_fb && pair_lr);

#if (ALIGN_MODE == 2)
        /* 阶段 A 要"前 + 右"都在；上路之后巡线只认前灰阶一块。 */
        pair_fr  = (uint8_t)(ok[ALIGN_F] && ok[ALIGN_R]);
        pair_use = setup_done ? ok[ALIGN_F] : pair_fr;
#else
        pair_use = (uint8_t)(pair_fb || pair_lr);
#endif

        /* --- 需要的灰阶都凑不齐 -> 慢慢转着找线 --- */
        if (pair_use) {
            acq = 0;
        } else if (acq < ALIGN_ACQ_AFTER) {
            acq++;                  /* 短时间掉线先保持不动，别一惊一乍地转 */
        }

        searching = (uint8_t)(acq >= ALIGN_ACQ_AFTER);
#if (ALIGN_MODE == 2)
        if (setup_done) searching = 0;   /* 上路之后掉线不原地打转，见阶段 B */
#endif

        if (searching) {
            if (!ALIGN_OBSERVE_ONLY) {
                Wheel_Mix(0, 0, (int16_t)(ALIGN_SPIN_DIR * ALIGN_SPIN_SPEED));
            }
            LED_RED   = ((n / 50) & 1) ? ALIGN_LED_ON : ALIGN_LED_OFF;
            LED_GREEN = ((n / 50) & 1) ? ALIGN_LED_OFF : ALIGN_LED_ON;
#if ALIGN_DEBUG
            if ((n % ALIGN_PRINT_EVERY) == 0) {
                printf("Align: SEARCHING (no pair visible), spinning\r\n");
            }
#endif
            n++;
            delay_ms(ALIGN_TICK_MS);
            continue;
        }

        /* --- 三通道。每个通道都要用到一整对灰阶，两个都有效才算得出来。
               只用其中一个的话，(E_F + 0) 会算成一半的横向误差、(0 - E_F) 会
               凭空算出一个转角，车就乱转了 —— 所以宁可这个通道先不开（保持 0）。 */
        u_lat = pair_fb ? (int16_t)(e[ALIGN_F] + e[ALIGN_B]) : 0;
        u_fwd = pair_lr ? (int16_t)(e[ALIGN_L] + e[ALIGN_R]) : 0;

#if ALIGN_YAW_FROM_LR
        u_yaw = pair_lr ? (int16_t)(e[ALIGN_L] - e[ALIGN_R]) : 0;
#else
        u_yaw = pair_fb ? (int16_t)(e[ALIGN_B] - e[ALIGN_F]) : 0;
#endif

#if (ALIGN_MODE == 2)
        /* ================== 模式 2：启动对准 -> 直走巡线 ================== */
        blob = 0;                       /* 只有阶段 B 会把它置起来 */
        if (!setup_done) {
            /* --- 阶段 A：只转，不挪 ---
               要让"前灰阶"和"右灰阶"的黑线同时落到各自探头中间。
               车心大致压在两条线的交叉点上时，这两块的读数都是同一个车头偏角
               造成的（前 ≈ -a*θ、右 ≈ -b*θ，a≈b），所以两块加起来当误差，
               一起归零。转反了就把 ALIGN_SIGN_SETUP 改成 -1。 */
            u_setup = pair_fr ? (int16_t)(e[ALIGN_F] + e[ALIGN_R]) : 0;
            w  = Align_Chan((int16_t)(ALIGN_SIGN_SETUP * u_setup), ALIGN_K_SETUP);
            vx = 0;
            vy = 0;

            /* 判据要用"控制律真正在调节的那个量"，也就是两块的和。
               不能要求两块**各自**都落进死区：控制律把和压在 ±ALIGN_DEADBAND
               之内时，完全可能一块 +6 一块 -6（和是 0、两块都偏），
               "各自都在死区内"这个条件就永远不成立，车就卡在阶段 A 一直不动。
               （这条判据能成立的前提是 ALIGN_SETUP_BAND >= ALIGN_DEADBAND。） */
            if (pair_fr &&
                (u_setup > -ALIGN_SETUP_BAND) && (u_setup < ALIGN_SETUP_BAND)) {
                setup_done = 1;             /* 前、右都摆正了，切阶段 B */
            }

            (void)u_lat;
            (void)u_fwd;
            (void)u_yaw;
        } else if ((ALIGN_B_FIX == 0) || ok[ALIGN_F]) {
            /* --- 阶段 B：固定速度直走 ---
               角上只有前方和右方两条臂，车尾那块灰阶下面**根本没有线**，
               所以这一路只认前面那一块。 */

            lost = 0;
            vx = ALIGN_CRUISE_SPEED;

#if (ALIGN_B_FIX == 0)
            /* 不纠偏：四个轮子给同一个速度。这一帧跟 MoveTest 第一步"前进"
               发下去的东西一模一样，所以用它单独验"底盘直走直不直"。 */
            vy = 0;
            w  = 0;

#elif (ALIGN_B_FIX == 1)
            /* 转向纠偏：朝线的方向拧。拧到位 = 车心回到线上 + 车头顺了线，两个
               自由度一起修。误差取自车头前方 a ≈ 115mm 处，前瞻大所以过阻尼。
               （一块横着摆的探头只有一个读数，只够修一个量。用平移修的话只能
                 把线挪到探头中间，管不着车头歪不歪 —— 车会横着蹭线走。）
               符号见 ALIGN_SIGN_YAW：2026-10-10 实测 +1 时车往左边越走越偏，
               已经翻成 -1。改了这里还发散，就再翻回来。 */
            w  = Align_Filt(f_yaw, Align_BChan((int16_t)(ALIGN_SIGN_YAW * e[ALIGN_F])));
            vy = 0;

#else
            /* 平移纠偏：把线挪回探头中间，车头不管。方向反了翻 ALIGN_SIGN_LAT。 */
            vy = Align_Filt(f_lat, Align_BChan((int16_t)(ALIGN_SIGN_LAT * e[ALIGN_F])));
            w  = 0;
#endif

            /* 压在横向线上（十字/丁字路口）：这一帧算出来的 E 是被那一整块黑
               拉偏的，**不是"线偏了"**，拿它转向会在路口上莫名其妙地拐一下。
               判据见 Strip_Blob。这时候纠偏清零，直着开过去。 */
            blob = Strip_Blob(raw[ALIGN_F]);
            if (blob) {
                vy = 0;
                w  = 0;
            }

            w_hold = w;         /* 记下这一帧的纠偏，丢线的时候照着它接着拧 */

            (void)u_fwd;
            (void)u_yaw;
            (void)u_lat;
        } else {
            /* 前灰阶没线了。**不要立刻停** —— 探头能看见的横向范围只有 ±28mm
               左右，稍微偏出去就丢线，一丢就停车会看起来像"传感器没数据车就不走"。
               所以先按最后那次的纠偏方向接着走 ALIGN_LOST_MAX 个周期，多半能把
               线找回来；还找不回来才停住等。 */
            (void)u_lat;
            (void)u_fwd;
            (void)u_yaw;

            if (lost < ALIGN_LOST_MAX) {
                lost++;
                vx = ALIGN_CRUISE_SPEED;
                vy = 0;
                w  = w_hold;
            } else {
                vx = 0;
                vy = 0;
                w  = 0;
                w_hold = 0;
            }
        }

        f_lat = vy;
        f_yaw = w;
        f_fwd = vx;
        (void)f_fwd;                    /* 这个模式不用纵向通道 */
        (void)f_lat;                    /* 阶段 B 走"转向"不走"平移"，横向量不滤波 */
        (void)f_yaw;

        /* 给 LED 用的"这一阶段要的灰阶到位了吗"：
           阶段 A 看"前 + 右"这一对；阶段 B 只要前灰阶有线就算到位（不纠偏那一档
           一个传感器都不看，直接算到位）—— 后灰阶要等车往前开过一个车身才压上线，
           拿它当条件会一路红灯。 */
        all_four = (uint8_t)(setup_done ? ((ALIGN_B_FIX == 0) ? 1 : ok[ALIGN_F])
                                        : pair_fr);

#elif ALIGN_ONLY_YAW
        /* ---------------- 模式 1：只旋转 ---------------- */
        (void)u_lat;
        (void)u_fwd;
        vy = 0;
        vx = 0;
        w  = Align_Filt(f_yaw, Align_Chan((int16_t)(ALIGN_SIGN_YAW * u_yaw), ALIGN_K_YAW));

        f_lat = vy;
        f_yaw = w;
        f_fwd = vx;

        (void)f_lat;
        (void)f_fwd;

    #if ALIGN_YAW_FROM_LR
        all_four = pair_lr;
    #else
        all_four = pair_fb;
    #endif

#else
        /* ------- 模式 0：三通道全开（只观察时这里算了但不驱动） ------- */
        vy = Align_Filt(f_lat, Align_Chan((int16_t)(ALIGN_SIGN_LAT * u_lat), ALIGN_K_LAT));
        vx = Align_Filt(f_fwd, Align_Chan((int16_t)(ALIGN_SIGN_FWD * u_fwd), ALIGN_K_FWD));
        w  = Align_Filt(f_yaw, Align_Chan((int16_t)(ALIGN_SIGN_YAW * u_yaw), ALIGN_K_YAW));

        f_lat = vy;
        f_yaw = w;
        f_fwd = vx;
#endif

        if (!ALIGN_OBSERVE_ONLY) {
            if ((vx == 0) && (vy == 0) && (w == 0)) {
                Wheel_Stop();
            } else {
                Wheel_Mix(vx, vy, w);
            }
        }

        /* --- LED --- */
        if (all_four && (vx == 0) && (vy == 0) && (w == 0)) {
            LED_RED   = ALIGN_LED_OFF;              /* 锁定：绿灯常亮 */
            LED_GREEN = ALIGN_LED_ON;
        } else if (all_four) {
            LED_RED   = ALIGN_LED_OFF;              /* 四个都看到线，正在修正 */
            LED_GREEN = ((n / 50) & 1) ? ALIGN_LED_ON : ALIGN_LED_OFF;
        } else if (vx != 0) {
            LED_RED   = ((n / 25) & 1) ? ALIGN_LED_ON : ALIGN_LED_OFF;
            LED_GREEN = ALIGN_LED_OFF;              /* 红灯快闪 = 丢线了还在按上次的纠偏走 */
        } else {
            LED_RED   = ALIGN_LED_ON;               /* 有灰阶没看到线，而且已经停住了 */
            LED_GREEN = ALIGN_LED_OFF;
        }

#if ALIGN_OBSERVE_ONLY
        /* 观察模式：红绿交替闪，一眼能看出现在不是在对准模式、没在驱动电机 */
        LED_RED   = ((n / 100) & 1) ? ALIGN_LED_ON : ALIGN_LED_OFF;
        LED_GREEN = ((n / 100) & 1) ? ALIGN_LED_OFF : ALIGN_LED_ON;
#endif

#if ALIGN_DEBUG
        /* 一行里按 F B L R 的顺序：先原码（两位十六进制），再位置读数 E（带符号），
           然后是三通道误差 lat/yaw/fwd，最后是实际下发量 vx/vy/w，
           末尾 g???? 是四路是否有效。 */
        if ((n % ALIGN_PRINT_EVERY) == 0) {
            char hx[4][3];
            char gv[5];
            char ph;

            for (i = 0; i < 4; i++) gv[i] = ok[i] ? '1' : '0';
            gv[4] = '\0';

            /* 现在在哪一段：A 启动对准 / B 上路直走 / S 转着找线 / x 停住 */
#if (ALIGN_MODE == 2)
            /* B = 正常巡线，C = 压在横向线上不转向，
               L = 丢线了但还在按上次的方向继续走，x = 丢太久停住了 */
            ph = searching ? 'S'
               : ((!setup_done) ? 'A'
               : (blob ? 'C'
               : ((lost != 0) ? ((vx != 0) ? 'L' : 'x') : 'B')));
#else
            ph = searching ? 'S' : 'M';
#endif

            printf("Align: F%s %d  B%s %d  L%s %d  R%s %d  | lat %d yaw %d fwd %d"
                   " | vx %d vy %d w %d  g%s  ph%c\r\n",
                   Hex2(hx[0], raw[0]), (int)e[0],
                   Hex2(hx[1], raw[1]), (int)e[1],
                   Hex2(hx[2], raw[2]), (int)e[2],
                   Hex2(hx[3], raw[3]), (int)e[3],
                   (int)u_lat, (int)u_yaw, (int)u_fwd,
                   (int)vx, (int)vy, (int)w, gv, ph);

            /* 真正下发到四个轮子的数（201 202 203 204）：
               "只有某几个轮子转"的时候，看这一行就够了。 */
            printf("       m201 %d  m202 %d  m203 %d  m204 %d\r\n",
                   (int)g_m[0], (int)g_m[1], (int)g_m[2], (int)g_m[3]);
        }
#endif

#if ALIGN_VOFA
        if ((n % ALIGN_VOFA_EVERY) == 0) {
            float vofa[7];
            vofa[0] = (float)e[ALIGN_F];
            vofa[1] = (float)e[ALIGN_B];
            vofa[2] = (float)e[ALIGN_L];
            vofa[3] = (float)e[ALIGN_R];
            vofa[4] = (float)vx;
            vofa[5] = (float)vy;
            vofa[6] = (float)w;
            JustFloat_Send(vofa, 7, USART1);
        }
#endif

        n++;
        delay_ms(ALIGN_TICK_MS);
    }
}
