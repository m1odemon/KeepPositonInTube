# Keep Position in Tube

本项目用于控制小球在水管（或管状轨道）中的位置。系统以 STM32F407
作为下位机主控，通过 CAN 总线控制 QD4310 无刷电机；电机驱动机械连杆改变
水管倾角，从而调节小球的运动方向和速度。

本目录提交的代码主要负责 F407 固件、无刷电机驱动、连杆/水管控制算法，
以及与树莓派上位机之间的通信与调试。

## 控制流程

```text
树莓派视觉/上位机
  │  小球位置、置信度、目标位置
  │  USART6（1152000-8-N-1）
  ▼
STM32F407
  │  位置环、速度环、管角度环
  │  CAN1（1 Mbit/s）
  ▼
QD4310 无刷电机 ── 连杆机构 ── 水管倾角 ── 小球位置
```

F407 同时把控制状态、电机角度、转速、电流和故障标志回传给树莓派，便于监视
与调试。

## 主要功能

- 解析树莓派发送的小球位置、置信度、目标位置和运行/急停命令。
- 根据小球位置和速度计算目标水管倾角。
- 根据电机角度与连杆标定表估算实际水管倾角。
- 通过 QD4310 电流指令驱动无刷电机，使连杆带动水管运动。
- 提供命令超时、电机反馈超时、角度限位、CAN 异常和急停等安全保护。
- 通过串口遥测回传控制状态，支持第一阶段硬件联调。

## 硬件与接口

| 模块 | 配置 |
| --- | --- |
| 主控 | STM32F407（大疆 C 板） |
| 电机 | QD4310 无刷电机，默认 CAN ID `1` |
| CAN | CAN1，`PD1/TX`、`PD0/RX`，经典 CAN 1 Mbit/s |
| 串口 | USART6，`PG14/TX`、`PG9/RX`，1152000-8-N-1 |
| 上位机 | 树莓派或电脑，通过串口发送控制帧、接收遥测 |

## 目录结构

| 目录 | 内容 |
| --- | --- |
| `App/` | FreeRTOS 控制任务、接口配置和调试任务 |
| `UserLib/BallBeam/` | 小球—水管控制器、连杆标定映射和通信协议 |
| `UserLib/QD4310/` | QD4310 无刷电机 CAN 驱动 |
| `Core/` | STM32CubeMX 生成的外设和系统代码 |
| `RPI/` | 树莓派/电脑端协议解析与第一阶段串口监视工具 |
| `Tests/` | 上位机协议测试 |
| `docs/` | 硬件接线、通信协议和阶段性验收说明 |
| `CubeMX_Rebuild/` | STM32CubeMX 重建工程备份 |

## 构建固件

需要安装 CMake、Ninja 和 Arm GNU Toolchain：

```powershell
cmake -S . -B build-h -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build-h -j
```

构建产物位于：

- `build-h/HBall.elf`
- `build-h/HBall.bin`
- `build-h/HBall.hex`

## 测试与串口监视

运行上位机协议测试：

```powershell
python -m pytest Tests
```

安装 `pyserial` 后，可以读取 F407 遥测：

```powershell
python RPI/monitor_stage1.py COM8
```

将 `COM8` 替换为实际串口。

## 首次上电与安全说明

当前固件保留了机构标定安全锁：

```cpp
// App/BallBeamConfig.h
config.calibration_valid = false;
```

在完成电机角度—水管倾角七点标定之前，不要将它改为 `true`。第一次烧录应使用
限流电源，并确保连杆未安装或可以自由运动；上电后电机不得主动转动。详细接线、
协议和验收步骤见 [`docs/H_BALL_STAGE1.md`](docs/H_BALL_STAGE1.md)。
