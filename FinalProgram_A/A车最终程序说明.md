# A车程序 FinalProgram_A —— 去程记录、投递与返程动态避障

> 更新日期：2026-10-31  
> 工程：`FinalProgram_A\FinalProgram_A.uvprojx`  
> 主程序：`FinalProgram_A\final_a_main.c`  
> 当前状态：运行时红蓝方选择已加入，尚未实车验证

## 0. 当前完整流程模式

源码当前默认：

```c
#define FINAL_FORWARD_ONLY_TEST 0
#define FINAL_FORWARD_TRANSLATION_ENABLE 1
#define FINAL_FORWARD_YAW_ENABLE 1
```

当前运行完整比赛流程：

- 初始化四个底盘电机和四块灰度；
- 上电默认红方，使用 `PD10` 按钮在红方和蓝方之间切换；
- 上电后只等待 `PD12` 上的 `TCRT5000` 启动信号，不使用语音启动；
- 初始化八路红外；
- 执行 BFS 路径规划；
- 执行四方向移动、平移/旋转纠偏和节点停车；
- 终点执行双舵机投递；
- 返程执行动态避障与 BFS 重规划。

当前 `FINAL_FORWARD_TRANSLATION_ENABLE = 1`、
`FINAL_FORWARD_YAW_ENABLE = 1`：

- 平移纠偏 `vy` 已开启；
- 旋转纠偏 `w` 同步开启；
- 前进旋转符号 `NODE_TRACK_YAW_SIGN = +1`，不做反向；
- 平移和旋转权重统一为 `{ 0, +7, +4, +1, -1, -4, -7, 0 }`。

串口每 `500 ms` 输出：

```text
Fwd: F=xx B=xx ok=x/x yaw=... lat=... vy=... w=...
```

`FINAL_FORWARD_TRANSLATION_ENABLE` 和 `FINAL_FORWARD_YAW_ENABLE`
只影响单独的连续前进调试循环，完整流程中不会调用该调试循环。

### TCRT5000 启动

队伍选择与启动传感器：

```text
队伍按钮：PD10，上拉输入，低电平按下
默认队伍：红方
红方显示：0000
蓝方显示：1111

信号：PD12
空闲：高电平
触发：低电平
确认：连续 3 次低电平
```

`PD10` 每按一次就在红方和蓝方之间切换：

```text
红方 -> 蓝方 -> 红方 -> ...
```

启动前电机保持停止。红方时红灯亮、数码管显示 `0000`；蓝方时绿灯亮、
数码管显示 `1111`。`TCRT5000` 连续确认启动后锁定当前队伍，数码管显示
`1234` 并进入 BFS 流程。
最终程序不初始化、不读取语音模块。

## 1. 程序目标

本工程把以下部分合并成一个独立比赛程序：

1. IRTest 的八路红外障碍检测；
2. NodeTest 的四方向节点间移动、灰度循迹和节点停车；
3. 8×5 节点地图上的 BFS 路径规划；
4. 去程实际路线记录、终点才开始的双舵机投递和返程路径恢复。

场地和坐标定义见：

```text
..\场地网格与四角巡线理解.md
```

基本模型：

```text
X = 0..7，共 8 列
Y = 0..4，共 5 行
车头固定朝 +Y
```

## 2. 红方和蓝方选择

上电默认红方：

```c
#define FINAL_TEAM_RED   0
#define FINAL_TEAM_BLUE  1
#define FINAL_DEFAULT_TEAM FINAL_TEAM_RED
```

在 `TCRT5000` 启动前按 `PD10` 可在红方和蓝方之间来回切换：

| 当前选择 | 数码管 | 起点 | 终点 |
|---|---|---|---|
| 红方 | `0000` | `(7,0)` | `(0,4)` |
| 蓝方 | `1111` | `(0,0)` | `(7,4)` |

启动后的数码管内容由下面这个宏指定，当前默认 `1234`：

```c
#define FINAL_RUN_DISPLAY 1234U
```

## 3. 完整任务流程

```text
上电初始化
  |
等待 1000 ms
  |
去程：当前节点四侧红外扫描
  |
更新 blocked[8][5]
  |
从当前节点到终点执行 BFS
  |
取得路径中的下一节点
  |
调用 NodeTest 前进/后退/左移/右移
  |
检测到下一节点后反向制动并停车
  |
等待 150 ms
  |
更新当前坐标并记录实际路线节点
  |
到达终点
  |
双舵机投递：持有 -> 中间 -> 投递 -> 中间 -> 持有
  |
返程：优先按去程路线逆序返回
  |
每个节点停车，四侧红外扫描
  |
下一节点被占用：原地等待，最多 5000 ms
  |
等待超时：把该节点记为固定障碍
  |
从当前节点用 BFS 找回家路线，等长路径优先去程节点
  |
回到起点，红灯常亮并锁停
```

去程每到一个节点都会重新规划。返程优先复用记录的实际路线；只有下一
节点持续被占用时才会更新障碍地图并重新 BFS。

两个面对面安装的舵机由 PE5/TIM9_CH1 和 PE6/TIM9_CH2 控制。

去程和返程都不初始化、不驱动舵机，PE5/PE6 不输出 PWM；只有到达终点后
才初始化 TIM9 并从持有状态开始投递。

| 状态 | PE5 / TIM9_CH1 | PE6 / TIM9_CH2 |
|---|---:|---:|
| 投递/释放 | `800 us` | `1700 us` |
| 中间 | `1250 us` | `1250 us` |
| 物品持有 | `1700 us` | `800 us` |

到达终点、调用 `Final_DeliveryTask()` 后执行：

```text
HOLD -> MID -> DELIVER -> MID -> HOLD -> RELEASE
```

每个状态停留 `1500 ms`。最后的 HOLD 完成后关闭 TIM9，把 PE5/PE6 改为
无上拉输入，释放舵机，然后开始返程；返程期间不再输出 PWM。

## 4. 四方向与坐标变化

| BFS 路径变化 | NodeTest 模式 | 说明 |
|---|---|---|
| `Y + 1` | `NODE_MODE_FORWARD` | 前进 |
| `Y - 1` | `NODE_MODE_BACK` | 后退 |
| `X - 1` | `NODE_MODE_LEFT` | 左移 |
| `X + 1` | `NODE_MODE_RIGHT` | 右移 |

路径层不执行 90° 转弯。NodeTest 的旋转量 `w` 只用于小幅姿态纠偏，
使车头继续接近 `+Y`。

A车程序 FinalProgram_A 实车标定后的旋转符号：

```c
#define NODE_TRACK_YAW_SIGN   (+1)  /* 前进，不做反向 */
#define NODE_BACK_YAW_SIGN    (+1)  /* 后退，前后组合修正后同前进 */
#define NODE_SIDE_YAW_SIGN    (-1)  /* 左移、右移 */
```

前后灰阶安装方向相反，因此直行时的误差组合为：

```c
平移误差 = 前向横向权重 - 后向横向权重；
旋转误差 = 前向旋转权重 + 后向旋转权重；
```

这样可对应实车数据规律：

```text
前1 + 后8 -> 平移修正
前1 + 后1 -> 旋转修正
```

前进和后退共用同一组前后误差组合；后退只把基础速度 `vx` 取反，
旋转纠偏符号在修正前后组合后同样恢复为 `+1`。

## 5. BFS 路径规划

障碍地图和原路优先表：

```c
static uint8_t Final_Blocked[8][5];
static uint8_t Final_Preferred[8][5];
```

- `0`：没有确认障碍，按可通过处理；
- `1`：红外已确认该节点有障碍，BFS 不进入。

由于每个相邻节点间移动代价都为 1，使用 BFS 可以得到最短节点路径。
当前 40 个节点的搜索量很小，不需要 D* Lite 或 RRT。

去程按 BFS 选择最短路径，并把每次实际到达的坐标记录到 `route_x/y[]`。
到达终点后，所有去程节点写入 `Final_Preferred[][]`。

### 节点间移动连续失败

去程和返程都使用同一规则：

```c
#define FINAL_TRAVEL_FAILURE_LIMIT 2U
```

一次失败可以是普通移动失败，也可以是驶出后检测到障碍并返回原节点。

1. 第一次失败后，车辆必须先回到原节点；
2. 普通失败时先检查车辆是否仍在原节点，否则反向循迹返回；
3. 中途障碍失败沿用已经完成的“返回原节点”动作，并随机等待
   `2000/4000/6000 ms`；
4. 从原节点向同一个目标发起第二次完整尝试；
5. 第二次仍然失败才把目标节点写入 `Final_Blocked[][]` 并重新 BFS；
6. 如果普通失败后无法确认或恢复到原节点，程序停车报错，不把目标节点
   误判为障碍。

串口会显示 `TRAVEL ATTEMPT FAILED ... cause=...` 和第二次失败后的
`TRAVEL FAILED 2 TIMES, block (x,y)`。

返程规则：

1. 以去程更新后的完整障碍地图为基础，从当前节点执行 BFS；
2. 每条等长路径中优先选择去程经过的节点；
3. 到达节点发现新障碍时，随机等待 `2000/4000/6000 ms` 后重新扫描；
4. 新障碍持续存在时写入固定障碍地图，并重新 BFS；
5. 障碍消失则继续按 BFS 路径移动。

返程检测到障碍时，先检查该方向的邻居节点是否已经在
`Final_Blocked[][]` 中：

- 如果只是第一轮去程已经记录过的障碍：不再等待，直接 BFS 绕行；
- 如果是本轮返程新出现的障碍：才随机等待 `2000/4000/6000 ms` 后复检。

### BFS 无解时逐节点回退

去程 BFS 找不到通往终点的路径时：

1. 沿实际行驶历史只退回一个节点；
2. 在新节点重新扫描四侧红外并更新障碍地图；
3. 再次执行 BFS；
4. 仍然无解则继续退一个节点；
5. 一直退到起点仍无解，才清空全部已知障碍并重新出发。

返程 BFS 找不到回家的路径时使用同样规则，沿返程历史每次退回一个节点，
直到退回终点。到达终点时仍无解，才清空障碍地图并重新开始返程；
不会再次执行投递动作。

主参数：

```c
#define FINAL_OBSTACLE_WAIT_MIN_MS   2000U
#define FINAL_OBSTACLE_WAIT_LEVELS   3U
#define FINAL_OBSTACLE_RANDOM_SALT   0xA5C35A3CUL
#define FINAL_OBSTACLE_RANDOM_PHASE  0U
#define FINAL_TRAVEL_FAILURE_LIMIT 2U
```

返程时如果 BFS 也找不到回家路径，程序停车并进入错误状态。

## 6. 八路红外障碍检测

使用 IRTest 实测引脚：

| 侧 | 探头 1 | 探头 2 |
|---|---|---|
| 前 | 前左 `FL = PD13` | 前右 `FR = PE12` |
| 后 | 后左 `BL = PB7` | 后右 `BR = PE4` |
| 左 | 左前 `LU = PD14` | 左后 `LD = PB8` |
| 右 | 右前 `RU = PE11` | 右后 `RD = PE3` |

输入为上拉、低电平有效：

```c
#define NODE_IR_ACTIVE_LOW 1
```

同一侧任意一只探头触发，就认为该侧有障碍。连续两个 `20 ms` 周期有效
才确认，约 40 ms 去抖。

E18-D80NK 返回的是数字触发信号，不是模拟距离。检测距离/灵敏度必须用
探头本体电位器标定；修改 `NODE_IR_CONFIRM` 或 `NODE_IR_POLL_MS` 不会
增加探测距离，它们只影响触发确认时间。

四侧映射到节点：

| 红外侧 | 标记的相邻节点 |
|---|---|
| 前 | `(X, Y+1)` |
| 后 | `(X, Y-1)` |
| 左 | `(X-1, Y)` |
| 右 | `(X+1, Y)` |

去程扫描到障碍后直接更新固定障碍地图。返程扫描时，先随机等待
`2000/4000/6000 ms` 再扫描一次；只有障碍仍存在时，才把对应节点坐标写入
`Final_Blocked[][]`。

`PB7` 在 IRTest 的历史测试中无响应，但后侧仍可由 `PE4` 触发。正式比赛
前仍应重新检查该路，避免单探头覆盖不足。

### 节点驶出后的 200 ms 连续检测

每段行程从节点驶出后，按当前移动方向每隔 `200 ms` 检测一次，
连续执行 5 次，总检测窗口为 `1000 ms`：

```c
#define NODE_MID_IR_CHECK_INTERVAL_MS 200U
#define NODE_MID_IR_CHECK_COUNT       5U
```

去程若检测到当前移动方向有障碍：

1. 立即按相反方向给速度 `-2`，持续 `400 ms` 制动；
2. 沿原路线反向循迹，返回刚出发的节点；
3. 不更新当前坐标；
4. 本次完整出行计为一次失败，但不立即封锁目标节点；
5. 随机等待 `2000/4000/6000 ms` 后，从原节点向同一个目标发起第二次尝试；
6. 第二次仍检测到障碍并返回，才把目标节点记为固定障碍并重新 BFS。

例如从 `(7,2)` 向前去 `(7,3)`，在途中检测到前方障碍：

```text
制动 -> 退回 (7,2) -> 随机等待 2000/4000/6000 ms -> 再走一次
第二次仍失败 -> Final_Blocked[7][3] = 1 -> 重新规划
```

返程同样执行 `200/400/600/800/1000 ms` 五次检测，并使用相同的两次
完整尝试规则：

1. 制动并返回刚出发节点；
2. 原地随机等待 `2000/4000/6000 ms`；
3. 从原节点向同一目标重新出发；
4. 第二次成功则继续，第二次仍失败才把目标节点记录为固定障碍并重新 BFS。

这样即使节点中心扫描漏检，只要驶出后的五次检测中任一次发现障碍，也不会直接
驶入该目标节点。

## 7. NodeTest 运动控制

本程序直接继承 NodeTest 的控制参数和轮速顺序：

```c
#define NODE_CRUISE            17
#define NODE_TICK_MS           20
#define NODE_TRACK_LAT_K       8
#define NODE_TRACK_LAT_MAX     3
#define NODE_TRACK_LONG_MAX    3
#define NODE_TRACK_YAW_K       8
#define NODE_TRACK_YAW_MAX     2
#define NODE_TRACK_DEADBAND    2
```

移动方向与灰度分工：

| 运动 | 循迹灰度 | 节点检测灰度 |
|---|---|---|
| 前进 | 前、后 | 左、右 |
| 后退 | 后、前 | 左、右 |
| 左移 | 左、右 | 前、后 |
| 右移 | 右、左 | 前、后 |

节点停车：

```c
#define NODE_BRAKE_SPEED 4
#define NODE_BRAKE_MS    400
```

负责节点检测的任意一侧出现以下任意两路全黑时停车：

```text
23、67
```

对应掩码：

```c
#define NODE_STOP_MASK_23 0x06U
#define NODE_STOP_MASK_67 0x60U
```

触发后按当前运动方向给四轮完整反向 `4` 速，持续 `400 ms`，随后停车。

## 8. 串口输出

USART1，`PA9`，`19200 8N1`。

启动示例：

```text
== A车 FinalProgram_A: RED team ==
Final: start=(7,0) goal=(0,4), outbound + delivery + return
Final: return wait=5000ms, prefer recorded route
```

节点扫描：

```text
Final: node=(7,0) IR F=0 B=0 L=0 R=0
```

移动：

```text
Final: OUT 1 (7,0) -> (7,1) FORWARD
Final: ARRIVED (7,1)
```

终点、投递与路线：

```text
Final: GOAL REACHED in 11 steps, route nodes=12
Final: OUTBOUND PATH nodes=12
Final: OUTBOUND[0]=(7,0)
...
Final: SERVO HOLD
Final: SERVO MID
Final: SERVO DELIVER
Final: SERVO MID
Final: SERVO HOLD
Final: SERVO RELEASE FOR RETURN
Final: RET 1 (0,4) -> (0,3) BACK
Final: RETURN ARRIVED (0,3)
Final: RETURN PATH nodes=12
Final: RETURNED TO START, return steps=11
```

返程遇到其他车辆：

```text
Final: WAIT FOR NODE CLEAR mode=1 waited=0 ms
Final: FIXED OBSTACLE (x,y), from node (cx,cy)
```

## 9. LED 状态

| 状态 | LED |
|---|---|
| 去程、返程运行中 | 绿灯常亮 |
| 终点双舵机投递动作 | 红灯常亮 |
| 返回起点并完成任务 | 红灯常亮 |
| 无路径、行驶超时或电机 I2C 失败 | 红灯闪烁 |

## 10. 编译与烧录

工程：

```text
FinalProgram_A\FinalProgram_A.uvprojx
```

命令行编译：

```powershell
$uv4 = 'D:\WORK\embedded development\Keil_AC5\UV4\UV4.exe'
Set-Location 'D:\WORK\2026江苏省机器人大赛\2024年江苏省机器人大赛-智慧快递\程序代码\智慧快递\FinalProgram_A'
Start-Process -FilePath $uv4 -ArgumentList @('-r','FinalProgram_A.uvprojx','-o','build-check.log','-j0') -Wait
```

固件：

```text
FinalProgram_A\Obj\FinalProgram_A.hex
```

编译结果：

```text
当前 `FINAL_FORWARD_ONLY_TEST = 0`：

```text
0 Error(s), Code=15862, RO=682, RW=132, ZI=1108
```
```

最终 HEX 使用默认红方配置。

Keil 的 Download 后如果车辆不运行，先按板载复位键。此前 ST-Link 下载后
曾把内核留在 Flash 下载算法中。

## 11. 第一次实车测试顺序

1. 确认 `FINAL_TEAM` 选对；
2. 车架空，确认四个方向移动和停车方向；
3. 逐个遮挡前、后、左、右红外，检查串口 `IR F/B/L/R`；
4. 把车放在起点节点；
5. 先不放障碍，确认去程、舵机投递动作、原路返程完整结束；
6. 在去程最短路径节点放障碍，确认前进阶段改道并记录新路线；
7. 在返程路线节点放障碍并保持超过 5 秒，确认标记固定障碍并 BFS 绕行；
8. 确认串口打印的 `OUTBOUND PATH` 和 `RETURN PATH` 坐标正确；
9. 再测试靠近障碍一侧的双探头和 `PB7`；
10. 最后测试红方和蓝方两个起点/终点配置。

## 12. 当前限制

- 本工程尚未实车验证；
- 红外检测距离必须覆盖一个网格间距；
- 当前假设障碍只在节点，不在边中；
- 当前没有多机器人通信；返程依赖红外避让和 A/B 不同种子的随机等待，
  降低两台车在节点同步停车后继续互相等待的概率；
- 当前在节点扫描，并在驶出后执行五次 `200 ms` 中途扫描；其余时间不做连续动态扫描；
- 如果终点节点被红外判为障碍，BFS 会报告无路径；
- 当前设备使用 NodeTest 的无编码器开环速度控制；
- 启动前通过 `PD10` 在红方和蓝方之间运行时切换；
- 当前已整合 PE5/PE6 双舵机投递动作，但尚未实车验证机械行程和舵盘姿态。

## 13. 主要文件

| 文件 | 作用 |
|---|---|
| `final_a_main.c` | A 车 BFS、红外、NodeTest 控制整合 |
| `FinalProgram_A.uvprojx` | Keil 工程 |
| `FinalProgram_A.uvoptx` | ST-Link / 调试配置 |
| `Obj\FinalProgram_A.hex` | 可烧录固件 |
| `..\场地网格与四角巡线理解.md` | 场地坐标和任务模型 |
