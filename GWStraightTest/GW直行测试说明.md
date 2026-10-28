# GWStraightTest —— 前后两块 GW 八路灰阶直行测试

> 工程：`GWStraightTest\GWStraightTest.uvprojx`  
> 传感器：前、后两块感为 GW 八路灰阶  
> 不使用：左、右灰阶

> 2026-10-28：本工程使用完整 L1-L8 八路巡线；速度控制和纠偏参数与
> `StraightTest` 一致，灰度模块为感为 GW，I2C 读取方式不同。

当前速度限制：

```c
#define STRAIGHT_CRUISE       10
```

速度控制与 `StraightTest` 相同，不再增加额外的四轮绝对限幅。

本测试程序没有启动按钮。上电初始化并等待 `STRAIGHT_START_DELAY_MS`
（默认 `1000 ms`）后，只要传感器读到有效线就会自动前进。

## 1. 硬件

| 传感器 | 引脚 | 地址 |
|---|---|---|
| 前向 GW | PB13=SCL，PB14=SDA | `0x4F` |
| 后向 GW | PD1=SCL，PD2=SDA | `0x4F` |

前、后模块都直接读取一个状态字节，不使用亚博 `0x12/命令0x01` 协议。

默认黑线极性：

```c
#define STRAIGHT_GW_BLACK_LEVEL 0
```

当前实车模块数字量与默认判断相反：原始 `0=黑、1=白`，因此使用 `0`
对原始字节取反后再用于巡线。

## 2. 数据有效性

当前使用完整 `L1-L8` 八路。前、后任意一块处理后为：

```c
0x00 或 0xFF
```

都认为八路全同，本周期判无效，不参与修正。

平移和旋转权重：

```c
{ +6, +4, +3, +1, -1, -3, -4, -6 }
```

前后误差组合：

```c
平移误差 = 前向横向权重 - 后向横向权重;
旋转误差 = 前向旋转权重 + 后向旋转权重;
```

对应实车规律：

```text
前1 + 后8 -> 平移修正
前1 + 后1 -> 旋转修正
```

宽线屏蔽使用完整八路：`123+45` 或 `678+45` 同时为黑时不修正。

## 3. 控制

平移修正：

```c
#define STRAIGHT_LAT_K     8
#define STRAIGHT_LAT_MAX   2
#define STRAIGHT_LAT_SIGN  (+1)
```

转弯修正：

```c
#define STRAIGHT_YAW_K     8
#define STRAIGHT_YAW_MAX   1
#define STRAIGHT_YAW_SIGN  (+1)
```

四轮输出：

```c
m201 = STRAIGHT_CRUISE + vy + w;
m202 = STRAIGHT_CRUISE - vy + w;
m203 = STRAIGHT_CRUISE + vy - w;
m204 = STRAIGHT_CRUISE - vy - w;
```

纯转弯时四轮变化量为 `(+w,+w,-w,-w)`，旋转中心在车体中心。

## 4. 数码管

数码管显示前向 GW 的 L2-L7 中识别为黑色的路号，最多显示4路：

- 例如 `2、4、6` 为黑：显示 `246`；
- 无黑线、读取失败或超过4路为黑：全部熄灭。

## 5. 串口

USART1，PA9，`19200 8N1`：

```text
GWStraightTest: front/back GW 8ch @0x4F
Straight: F=raw->bits(...) B=raw->bits(...) ...
```

## 6. 测试

1. 打开 `GWStraightTest\GWStraightTest.uvprojx`；
2. Rebuild；
3. Download 并复位；
4. 等待启动延时后自动前进；
5. 观察直行平移和转弯修正；
6. 若黑白相反，修改 `STRAIGHT_GW_BLACK_LEVEL`；
7. 若平移方向相反，翻转 `STRAIGHT_LAT_SIGN`；
8. 若转弯方向相反，翻转 `STRAIGHT_YAW_SIGN`。
