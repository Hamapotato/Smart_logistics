# Yahboom YB-MUX04-2.0 八路巡线模块协议

> 整理日期：2026-10-28  
> 官方仓库：`YahboomTechnology/8-Channel-tracking-module`

## 1. 官方资料

- [模块使用与协议](https://github.com/YahboomTechnology/8-Channel-tracking-module/blob/main/0.%208-channel%20line%20tracking%20module%20usage/8-channel%20line%20tracking%20module%20usage.pdf)
- [STM32-I2C 读取说明](https://github.com/YahboomTechnology/8-Channel-tracking-module/blob/main/1.%20Single%20board%20tutorials/2.STM32/STM32-I2C.pdf)
- [STM32-IO 读取说明](https://github.com/YahboomTechnology/8-Channel-tracking-module/blob/main/1.%20Single%20board%20tutorials/2.STM32/STM32-IO.pdf)
- [官方 STM32 源码下载页](https://drive.google.com/drive/folders/1cc_PeYRXZpG-Iv5f2GC5bqu01pw8_KGs)

本地配套资料：

```text
F:\BaiduNetdiskDownload\八路巡线红外传感器\0.8路巡线模块的使用方法\8路巡线模块的使用方法.pdf
```

## 2. 支持的接口

模块支持三种读取方式：

1. 八路独立 IO；
2. 串口；
3. I2C。

本项目使用 I2C：

```text
7 位地址：0x12
数据寄存器：0x30
校准控制寄存器：0x01（只写）
```

**0x01 不是数据寄存器。**

## 3. I2C 数据格式

官方 `deal_IRdata()` 实现：

```c
u8 IRbuf;
HAL_I2C_Mem_Read(&hi2c2, (0x12 << 1), 0x30,
                 I2C_MEMADD_SIZE_8BIT, &IRbuf, 1, 10);

x1 = (IRbuf >> 7) & 1;
x2 = (IRbuf >> 6) & 1;
x3 = (IRbuf >> 5) & 1;
x4 = (IRbuf >> 4) & 1;
x5 = (IRbuf >> 3) & 1;
x6 = (IRbuf >> 2) & 1;
x7 = (IRbuf >> 1) & 1;
x8 = (IRbuf >> 0) & 1;
```

位序：

| 数据位 | 探头 |
|---|---|
| bit7 | X1 |
| bit6 | X2 |
| bit5 | X3 |
| bit4 | X4 |
| bit3 | X5 |
| bit2 | X6 |
| bit1 | X7 |
| bit0 | X8 |

如果要从 X1 到 X8 顺序存入 bit0 到 bit7，可使用：

```c
bits = reverse(raw);
```

但模块正面与背面标号方向相反。官方实物图显示：

```text
正面从左到右：L1, L2, L3, L4, L5, L6, L7, L8
背面从右到左：X1, X2, X3, X4, X5, X6, X7, X8
```

因此物理位置与探头编号的对应关系是：

| 正面位置（左到右） | 探头标号 | 原始数据位 |
|---|---|---|
| L1 | X8 | bit0 |
| L2 | X7 | bit1 |
| L3 | X6 | bit2 |
| L4 | X5 | bit3 |
| L5 | X4 | bit4 |
| L6 | X3 | bit5 |
| L7 | X2 | bit6 |
| L8 | X1 | bit7 |

## 4. 黑白极性

官方示例：

```text
X1 亮灯（在黑线上），其他探头灯不亮：
B01111111 = 0x7F = 127
```

PDF 表格中的 `B01111111 -> 0x8F` 存在笔误：`01111111`
对应 `0x7F`，十进制也是 `127`。第二例
`B10101111 = 0xAF = 175` 的二进制、十六进制和十进制一致。

因此原始数据位表示：

| 探头状态 | 原始 bit |
|---|---|
| 黑线 | `0` |
| 白线 | `1` |

若模块严格按该参考位序输出，理论换算为：

```c
bits = (uint8_t)(~raw);
```

当前实车模块的输出相对该参考位序偏移一位。实测直接使用
`~raw` 时，L1 黑会显示 7、L2 黑会显示 6。因此测试工程采用
StraightTest 已实测的换算：

```c
uint8_t v = reverse(raw);
v = (uint8_t)((v >> 1) | (v << 7));
bits = (uint8_t)(~v);
```

## 5. 校准

官方要求：

1. 每次上电先等待至少 `20 s`，让探头稳定；
2. 长按模块上的 `key1`，等红灯常亮，进入校准；
3. 按实际运行高度放在黑线上，稳定 3 秒；
4. 短按 `key1` 记录黑色；
5. 放在白线上，稳定 3 秒；
6. 再短按 `key1` 记录白色；
7. 校准成功后红灯熄灭并进入检测模式。

校准结果会保存，断电后仍有效。改变场地材质、环境光或安装高度后应重新校准。

## 6. 安装高度

官方推荐探头距离地面约：

```text
18 mm
```

白底场地建议使用哑光黑线；如果黑线透出背景，可增加一层黑线。

## 7. 常见“某一路不亮”

官方针对单路识别差还提供了机械调整：

- 白底上某路一直不亮：把该路发射管和接收管稍微掰开；
- 黑底上某一路一直亮：把该路发射管和接收管稍微夹紧。

但本次四个模块都表现为 X8 无数据，而 I2C 通信正常，因此首先修正了软件协议。

## 8. 原代码问题

旧程序使用：

```c
寄存器 0x01
reverse(raw)
循环移位
取反
```

官方实现是：

```c
寄存器 0x30
x1 = bit7 ... x8 = bit0
```

`0x01` 实际是校准开关，不是状态数据。旧位序换算还会把 X8 的处理错位，因此四个模块都出现“第 8 路无效”的相同现象。

## 9. GreyDisplayTest 修正

GreyDisplayTest 已改为：

```c
#define GREY_YB_DATA_REG      0x30U
#define GREY_YB_CALIB_REG     0x01U
#define GREY_WARMUP_MS        20000U

v = reverse(raw);
v = (v >> 1) | (v << 7);
bits = (uint8_t)(~v);
```

程序只读取 `0x30`，不会误写校准寄存器。换算同时处理了：

1. 官方原始数据是黑色为 0、白色为 1，需要反相；
2. 正面 L1..L8 与背面 X8..X1 方向相反；
3. 当前实物模块相对参考位序存在一位偏移。
