# Codex 对话交接：H 题一维滚球控制

> 用途：在新的 Codex 窗口中恢复本次对话的目标、设计决定、代码状态和下一步工作。
>
> 最后更新：2026-07-30。仓库与实物状态可能继续变化，开始工作前必须重新检查源码和 Git 状态。

## 1. 新窗口启动指令

把下面这段话发给新的 Codex 窗口：

> 请先完整阅读仓库根目录的 `CODEX_BALL_CONTROL_HANDOFF.md`，再阅读 `PROJECT_CONTEXT.md` 和文中列出的滚球文档。以当前工作区源码为准，先执行 `git status --short`，不要覆盖、回滚或提交已有的用户改动。先向我汇报当前实现、阻塞项和建议的下一步，再按我选择的执行机构路线继续。每次由你产生的更改都要单独 Git 提交，只暂存你实际修改的文件。

推荐新窗口首先执行：

```powershell
git status --short
git log -8 --oneline
Get-Content -Raw -Encoding UTF8 CODEX_BALL_CONTROL_HANDOFF.md
Get-Content -Raw -Encoding UTF8 PROJECT_CONTEXT.md
Get-Content -Raw -Encoding UTF8 project\README_BALL_SYSTEM_DESIGN.md
Get-Content -Raw -Encoding UTF8 project\README_BALL_RPI_TEAM.md
Get-Content -Raw -Encoding UTF8 project\README_TASK3_BALL_OFFLINE.md
```

## 2. 用户的最终目标

完成 2026 年 H 题车载一维滚球系统，包括：

- 小车底盘、循迹、比赛任务状态机；
- 固定在车架正上方的控制摄像头；
- 树莓派视觉识别和下位机通信；
- 钢球位置闭环；
- 分别尝试 B4 舵机和 STM32F407 + QD4310 无刷电机两种水管执行机构；
- 独立图传模块向电脑提供监视画面；
- 机械、电控、程序和调参全过程逐步完成。

当前最明确的静止滚球 Task3 指标：

```text
小车静止
O 点 → +50 mm → -50 mm 并稳定
总时间不超过 5 s
两个目标点最大位置误差绝对值不超过 10 mm
```

比赛规则允许交流和使用 AI。

## 3. 已确认的实物结构

- 控制摄像头固定在车架正上方，不随水管倾斜；
- 树莓派固定在车上；
- 水管控制摆角预计较小，摄像头视野变化可通过每帧水管参考识别补偿；
- 题目所需图传准备使用独立模块完成，不占用滚球控制 UART；
- 当前水管上没有陀螺仪；
- 舵机连短杆约在 90° 时对应水管水平；
- 机械模型已经完成，但本对话中尚未提供短杆、长杆的精确长度及实测角度—脉宽数据；
- 第一版不要求增加水管 IMU。若后续必须做角度闭环，优先考虑直接测量水管相对车体角度的磁编码器或电位器。

## 4. 两条执行机构路线必须分开

### 4.1 TI 舵机路线

```text
控制摄像头
  → 树莓派：钢球/水管识别、毫米坐标和置信度
  → UART2
  → MSPM0G3507
  → 位置—速度串级控制
  → 水管目标角
  → 三点角度—脉宽标定
  → B4 舵机
  → 连杆和水管
```

明确含义：“树莓派和 TI 芯片通信然后控制舵机”只指这条路线。

### 4.2 STM32F407 无刷路线

```text
控制摄像头
  → 树莓派
  → USART6
  → STM32F407
  → 位置环、速度环、水管角度/电流控制
  → CAN
  → QD4310 无刷电机
  → 连杆和水管
```

该路线不经过 TI，也不控制 B4 舵机。对应独立仓库：

```text
D:\Codex_TI_NEW\工程模板\QGimbal
```

每次实验只能让一套执行机构与水管刚性连接并上电。不得把两块下位机的 TX 输出并联。

## 5. 当前采用的控制职责

### 5.1 树莓派

树莓派负责：

- 采集图像，并在采图时立即记录单调时间戳；
- 检测钢球中心和本帧水管参考；
- 将钢球投影到水管轴；
- 输出相对水管的 `position_mm`；
- 输出 `confidence`、`sequence` 和 `capture_timestamp_us`；
- 根据选中的后端打包 UART 帧；
- 保存调试画面、视觉日志；
- F407 路线还需接收并记录下位机遥测。

树莓派第一版不直接计算舵机 PWM、无刷电机电流或电机转速。

### 5.2 TI 舵机控制

当前 TI 控制器是位置—速度串级：

```text
位置 PI → 目标钢球速度
速度 PD → 目标水管角度
角度—脉宽标定 → 舵机脉宽
舵机内部位置环 → 舵机轴位置
```

位置和速度都来自摄像头，因此没有水管陀螺仪也能实现串级控制。它不等于独立高速水管角度环。

第一轮调试：

- `Ki_pos = 0`；
- `Kd_vel = 0`；
- 车体加速度前馈为 `0`；
- 先确认方向和小角度安全，再逐步增加比例增益。

### 5.3 F407 无刷控制

F407 使用电机反馈和连杆标定估计水管角度，再通过电流指令控制 QD4310。当前有硬安全锁：

```cpp
config.calibration_valid = false;
```

完成电机零位及七点“电机相对角—实际水管角”实测前，不得解除。

## 6. 坐标和标定决定

实物水管应标记：

```text
P- —— O —— P+
```

- `O` 为控制零点，`position_mm = 0`；
- 钢球向 `P+` 移动时 `position_mm` 增大；
- 正水管角使钢球向 `P+` 加速；
- F407 路线把 `P+` 标在电机端；
- TI 舵机也按同一物理方向标定。

树莓派必须发送钢球**相对水管**的毫米位置，不能发送原始 `pixel_x`。

摄像头不随水管倾斜，所以每帧应重新寻找水管两端/标记并建立管轴。至少完成 `P- / O / P+` 三点标定；存在畸变或透视时使用去畸变、单应变换或多点查表。

关键验收：

> 水管发生允许的小角度倾斜、钢球相对水管不动时，树莓派的 `position_mm` 应基本不变。

## 7. 树莓派—TI 舵机串口协议

完整、可直接交给树莓派队友的 TI 舵机接口文档：

[project/README_BALL_RPI_TEAM.md](project/README_BALL_RPI_TEAM.md)

该接口文档只允许描述以下链路：

```text
树莓派 → MSPM0G3507 → B4 舵机
```

其他执行机构属于独立工程，不得再混入该接口文档。

### 7.1 固定协议

```text
波特率：115200，8-N-1，3.3 V TTL
接线：Pi TX → MSPM0 B16/UART2_RX
      Pi RX ← MSPM0 B15/UART2_TX
帧长：21 字节
帧头：A5 5A
内容：version、sequence、capture_timestamp_us、
      position_mm、confidence、CRC16
CRC：CRC-16/CCITT-FALSE，覆盖偏移 2..18
```

当前 TI 默认：

- 有效位置范围 `-120~+120 mm`；
- 最低置信度 `0.70`；
- 视觉 `dt` 有效范围 `5~150 ms`；
- 连续 3 个有效帧后启动；
- 180 ms 无有效帧触发视觉超时。

## 8. TI 仓库已经完成的代码

核心文件：

| 文件 | 当前作用 |
|---|---|
| `project/code/ball_control.c/.h` | 位置/速度/加速度估计、位置—速度串级、稳定判定和安全状态 |
| `project/code/ball_actuator.c/.h` | 水管角度到 B4 脉宽的三点映射、硬限幅、变化率限制 |
| `project/code/ball_vision_link.c/.h` | TI 21 字节帧解析、CRC 和完整帧快照 |
| `project/code/ball_balance_app.c/.h` | 组合参考接口；当前不参加实际 Task3 调用链 |
| `project/code/task3_ball.c/.h` | `+50 mm → -50 mm` 比赛状态机 |
| `project/tests/ball_control_host_test.c` | 主机侧控制和安全测试 |

Task3 已加入 Keil 和 5 ms 调度：

```text
READY
  → WAIT_SENSOR
  → TO_PLUS_50
  → TO_MINUS_50
  → DONE_HOLD
```

左右底盘电机在静止滚球 Task3 中保持关闭，B4 回安全中位。

## 9. 当前最重要的代码阻塞

TI 的 `UART2_IRQHandler()` 当前只允许电脑 PID 调参任务解析字符；其他任务收到的 UART2 数据会被主动清空。

因此目前：

- 树莓派可以生成并发送正确的 TI 21 字节帧；
- `ball_vision_link` 可以解析该帧；
- Task3 有 `task3_ball_submit_measurement()` 接口；
- 但 UART2 ISR 尚未把三者接起来；
- Task3 实机仍停在 `WAIT_SENSOR`。

未来 TI 接入应遵守：

```c
/* UART2 ISR：只收字节、解析并发布完整帧。 */
ball_vision_link_rx_byte_from_isr(&vision_link, rx_byte, now_us);

/* 5 ms 普通任务上下文：取出完整帧后提交。 */
if (ball_vision_link_take_latest(&vision_link, &frame)) {
    task3_ball_submit_measurement(&frame.measurement);
}
```

不要在 UART ISR 中执行 PID、舵机控制、OLED 或阻塞操作。

接入前必须设计好 UART2 与电脑 PID 调参任务的互斥方式；不能让 ASCII 协议和二进制协议同时解析同一数据流。

## 10. 仍待实测的参数

### 10.1 树莓派视觉

- 摄像头设备、分辨率、实际 FPS；
- 曝光、增益、补光；
- 水管 ROI；
- 可滚动有效长度；
- `P- / O / P+` 像素和毫米坐标；
- 钢球半径/面积/圆度范围；
- 置信度算法和误检阈值；
- 真实处理延迟、帧间隔抖动和失视率。

### 10.2 TI 舵机和连杆

- 水管真实水平时的舵机脉宽；
- 安全正、负最大水管角及对应脉宽；
- 正方向；
- 连杆是否碰撞、过中心、进入死区；
- 回差与相同命令的重复性。

当前默认：

```text
-5° → 1350 µs
 0° → 1500 µs
+5° → 1650 µs
```

这些只是软件占位值，不是成品标定值。

### 10.3 F407 无刷和连杆

- QD4310 的实际 CAN ID；
- 电机绝对零位；
- 七组电机相对角—水管角；
- 正方向；
- 机械限位；
- 安全电流上限；
- 标定后是否允许解除 `calibration_valid=false`。

## 11. 推荐的下一步

根据硬件准备程度只选择一项：

### 选项 A：树莓派视觉工程

依据 `project/README_BALL_RPI_TEAM.md` 建立可运行的树莓派工程，实现：

- 相机采集；
- 水管参考检测；
- 钢球检测；
- 毫米标定；
- `confidence`；
- TI 21 字节视觉协议；
- 115200 UART 发送；
- CSV/视频日志；
- 协议单元测试。

### 选项 B：TI UART2 接入

在不破坏电脑 PID 调参任务的前提下，把 `ball_vision_link` 接入 UART2 ISR，并在 5 ms 上下文提交给 Task3。先只验证收帧、CRC、超时和 OLED 状态，舵机保持安全中位。

### 选项 C：TI 舵机机械标定

机构搭好后，先断开滚球闭环，测量水平中位、正负小角度、方向、碰撞边界和回差，再填写 `ball_actuator` 配置。

### 选项 D：F407 无刷联调

进入 `QGimbal` 仓库，先完成 CAN、电机反馈和遥测，再做七点机构标定。标定锁解除前不要运行滚球闭环。

建议顺序：

```text
视觉离线标定
  → 串口协议验帧
  → 单执行机构开环与安全标定
  → 低增益静态中心
  → +50/-50 mm
  → 小车低速运动
  → 完整比赛任务
```

## 12. 参考资料

仓库内：

- [总项目上下文](PROJECT_CONTEXT.md)
- [滚球整体方案](project/README_BALL_SYSTEM_DESIGN.md)
- [树莓派—TI 舵机接口规范](project/README_BALL_RPI_TEAM.md)
- [Task3 当前离线状态](project/README_TASK3_BALL_OFFLINE.md)
- [树莓派通信现状](project/README_RPI_COMM.md)

用户提供的外部资料：

```text
C:\Users\18371\Downloads\车载平衡滚球运动控制系统（H题）.pdf
C:\百度网盘\2026电赛\H题目\2026全国大学生电子设计竞赛H题浅析-逐飞科技.docx
C:\Users\18371\OneDrive\桌面\open-ed-master\EDC\2024电赛训练\
2017年B题板球控制系统\一维管槽滚球控制_README.md
```

## 13. Git 和协作注意事项

- 用户要求：每次 Codex 更改后都创建 Git 提交；
- 只暂存本次实际修改的文件；
- 工作区经常有其他窗口或用户的未提交修改，绝不能批量暂存；
- 不执行 `git reset --hard`、`git checkout --` 或清理其他人的文件；
- 修改前后都检查 `git status --short`；
- 编译产物不作为源码修改提交；
- 中文旧源码可能存在编码混乱，不要仅为“修复显示”批量重写源文件。

创建本文档时观察到：

- `project/README_BALL_RPI_TEAM.md` 已由本对话重写为纯 TI 舵机接口；以最新工作区和提交为准；
- `PROJECT_CONTEXT.md`、底盘代码和其他 README 存在本对话之外的未提交修改；
- `CODEX_TASK_HANDOFF.md` 和 `project/README_TASK1_H_TRACK_PID.md` 是其他工作产生的未跟踪文件，不要覆盖或顺带提交；
- 新窗口必须重新运行 `git status --short`，以实际结果为准。

## 14. 本次对话已形成的关键结论

1. 树莓派发送相对水管的毫米位置，不只发送像素，也不直接发舵机 PWM。
2. 无水管陀螺仪仍可使用摄像头位置—速度串级。
3. 普通舵机自身有轴位置内环，第一版水管角度环不是必需项。
4. 连杆长度未知时不能靠理论模型直接得到可靠水管角，必须现场标定。
5. 摄像头固定在车架正上方可行，但必须每帧识别水管参考并保证 ROI 覆盖所有姿态。
6. 树莓派—TI 舵机接口文档不得混入另一套执行机构的协议或参数。
7. 不同执行机构路线必须分别维护文档、协议和调试记录。
8. 在视觉坐标、方向和安全限位通过前，不开始正式 PID 调参。
