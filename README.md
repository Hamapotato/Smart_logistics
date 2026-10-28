# Smart_logistics

STM32F407ZGT6 智慧快递机器人 Keil 工程、测试程序和比赛最终控制程序。

## 当前最终程序

[FinalProgram](FinalProgram/FinalProgram.uvprojx) 是当前整合版比赛程序，包含：

- 8×5 节点矩形网格上的 BFS 路径规划；
- `PD12` 上的 TCRT5000 唯一启动控制，不使用语音启动；
- 红方、蓝方起点和终点切换；
- IRTest 八路红外障碍检测；
- NodeTest 四方向循迹、节点停车和反向制动；
- 去程实际路线记录；
- 去程和返程都不驱动舵机；到终点后才启动双舵机投递，序列结束后释放舵机；
- 原路优先返程，遇到其他车辆时等待、标记固定障碍并重新 BFS。

> 当前源码启用了 `FINAL_FORWARD_ONLY_TEST = 0`，
> `FINAL_FORWARD_TRANSLATION_ENABLE = 1`、`FINAL_FORWARD_YAW_ENABLE = 1`，
> 运行完整流程，包括 BFS、红外、四方向移动、停车、双舵机投递和返程。

说明：[FinalProgram/最终程序说明.md](FinalProgram/最终程序说明.md)

上电默认红方。在 `TCRT5000` 启动前按 `PD10` 按钮可在红方和蓝方之间切换：

```text
红方：数码管 0000
蓝方：数码管 1111
启动后：数码管 1234
```

## 场地与硬件资料

- [六路巡线规则](六路巡线规则.md)
- [场地网格与四角巡线理解](场地网格与四角巡线理解.md)
- [接线清单](接线清单.md)
- [硬件清单](硬件清单.md)
- [烧录指南](烧录指南.md)
- [幻尔四路编码电机驱动板资料](幻尔四路编码电机驱动板资料.md)
- [循迹验证与后续工作](循迹验证与后续工作.md)
- [测试工程更新说明](测试工程更新说明.md)

## 测试工程

| 工程 | 用途 | 说明 |
|---|---|---|
| [MoveTest](MoveTest/MoveTest.uvprojx) | 四轮方向、平移和旋转组合 | [说明](MoveTest/底盘运动测试说明.md) |
| [TraceTest](TraceTest/TraceTest.uvprojx) | 四方向单灰阶循迹 | [说明](TraceTest/循迹测试说明.md) |
| [IRTest](IRTest/IRTest.uvprojx) | 八路红外和反方向避障 | [说明](IRTest/红外避障测试说明.md) |
| [SpeedTest](SpeedTest/SpeedTest.uvprojx) | JGB37-520 编码器速度闭环 | [说明](SpeedTest/速度闭环测试说明.md) |
| [StraightTest](StraightTest/StraightTest.uvprojx) | 前后双灰度直行 | [说明](StraightTest/直行测试说明.md) |
| [FourDirStraightTest](FourDirStraightTest/FourDirStraightTest.uvprojx) | 四方向单轴直行与纠偏开关 | [说明](FourDirStraightTest/四方向直行测试说明.md) |
| [GWStraightTest](GWStraightTest/GWStraightTest.uvprojx) | GW 双灰度直行 | [说明](GWStraightTest/GW直行测试说明.md) |
| [NodeTest](NodeTest/NodeTest.uvprojx) | 节点间移动、停车和固定任务序列 | [说明](NodeTest/节点测试说明.md) |
| [ServoTest](ServoTest/ServoTest.uvprojx) | PE5/PE6 双舵机镜像与安装测试 | [说明](ServoTest/舵机测试说明.md) |
| [GreyDisplayTest](GreyDisplayTest/GreyDisplayTest.uvprojx) | 单方向八路灰度黑线路号数码管显示 | [说明](GreyDisplayTest/数码管寻线显示说明.md) |

## 源码约定

- 主要使用 Keil MDK AC5 和 STM32F4 Standard Peripheral Library；
- 旧 `.c/.h` 文件通常为 GBK + CRLF；
- Markdown 文档为 UTF-8；
- `Obj/`、`*.hex`、`*.axf`、`*.o`、`*.map` 和构建日志不提交；
- 每台机器重新编译后会在各自工程的 `Obj/` 下生成 HEX。

## 2026-10-28 目录迁移

不需要的历史工程、完整公共源码副本和旧模板已移到仓库外部的：

```text
..\old
```

其中包括 `Template.uvprojx`、`Align`、`Test_Grey`、`GreyCalibrate`、
日志和个人调试配置等。

当前 10 个工程所需的 90 个共享文件已从 `old` 选择性恢复到根目录，包括：

- 工程明确引用的 `APP`、`Libraries`、`Public`、`User` 源文件；
- 21 个 include 目录中的必要头文件；
- `APP`、`User`、`Libraries`、`Public` 的原始相对路径保持不变。

2026-10-28 已对 10 个工程逐一执行 Keil 全量 Rebuild，全部 `0 Error(s)`。
