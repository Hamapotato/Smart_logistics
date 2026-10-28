# GWStraightTest —— 前后两块 GW 八路灰阶直行测试

> 工程：`GWStraightTest\GWStraightTest.uvprojx`  
> 传感器：前、后两块感为 GW 八路灰阶  
> 不使用：左、右灰阶

> 2026-10-28 同步：所有巡线权重只使用 L2-L7，L1/L8 通过 `0x7E`
> 掩码屏蔽；六路全白 `0x00`、全黑 `0x7E` 均判无效。

## 1. 硬件

| 传感器 | 引脚 | 地址 |
|---|---|---|
| 前向 GW | PB13=SCL，PB14=SDA | `0x4F` |
| 后向 GW | PD1=SCL，PD2=SDA | `0x4F` |

前、后模块都直接读取一个状态字节，不使用亚博 `0x12/命令0x01` 协议。

默认黑线极性：

```c
#define STRAIGHT_GW_BLACK_LEVEL 1
```

如果黑白电平相反，改成 `0`。

## 2. 数据有效性

当前只使用 L2-L7 六路，L1/L8 通过 `0x7E` 掩码屏蔽。前、后任意一块
六路数据处理后为：

```c
0x00 或 0x7E
```

都认为中间六路全同，本周期判无效，不参与修正。

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

- 例如 `1、4、5、8` 为黑：显示 `1458`；
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
