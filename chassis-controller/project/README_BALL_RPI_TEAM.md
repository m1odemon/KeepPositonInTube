# H 题一维滚球：树莓派视觉—TI 舵机接口规范

本文专门交给树莓派队友使用，只定义以下控制链路：

```text
固定在车架正上方的控制摄像头
  → 树莓派识别钢球和水管
  → UART
  → TI MSPM0G3507
  → 位置—速度串级控制
  → B4 舵机
  → 连杆改变水管倾角
```

本文不包含其他下位机或执行器方案。

## 0. 一页版接口结论

树莓派每获取一张新图像，只向 TI 板发送四项视觉测量：

```text
sequence
capture_timestamp_us
position_mm
confidence
```

TI 板负责：

- 校验串口帧；
- 判断帧是否重复、乱序或超时；
- 对位置滤波并估计钢球速度；
- 执行位置 PI—速度 PD 串级控制；
- 计算目标水管角；
- 将目标水管角换算为 B4 舵机脉宽；
- 执行限幅、变化率限制、失视回中和故障保护。

树莓派不发送：

- 原始 `pixel_x` 代替毫米坐标；
- 目标水管角；
- 舵机角度；
- 舵机脉宽或 PWM；
- PID 参数；
- 虚假的旧位置。

固定通信参数：

```text
方向：树莓派 → MSPM0G3507
UART：115200 baud，8 data bits，no parity，1 stop bit，无流控
电平：3.3 V TTL
帧长：21 字节
字节序：小端
浮点：IEEE-754 float32
CRC：CRC-16/CCITT-FALSE
```

## 1. 双方职责

### 1.1 树莓派负责

每一帧按以下顺序处理：

```text
获取图像并立即记录采集时间
  → 裁剪水管 ROI
  → 检测本帧水管两端/参考标记
  → 建立本帧水管轴线
  → 检测钢球中心
  → 将钢球中心投影到水管轴线
  → 像素/几何坐标换算为 position_mm
  → 计算 confidence
  → 打包 21 字节 V1 帧
  → 串口发送一次
  → 保存 CSV 和调试画面
```

还要完成：

- 固定相机曝光、增益、分辨率和帧率；
- 保存现场几何标定参数；
- 遮挡、无球、严重反光时主动降低 `confidence`；
- 用单调时钟产生采集时间戳；
- 保证 `sequence` 单调递增；
- 统计 FPS、处理延迟、串口发送耗时和检测失败率。

### 1.2 TI 板负责

- 初始化 UART2；
- 在 UART2 接收中断中逐字节交给 `ball_vision_link`；
- 只在普通 5 ms 控制上下文取出完整测量；
- 调用 `task3_ball_submit_measurement()`；
- 在本地状态机设置 `0 mm`、`+50 mm`、`-50 mm` 等目标；
- 运行 `ball_control` 和 `ball_actuator`；
- 输出 B4 舵机 PWM；
- 在低置信度、视觉超时、位置越限或人工停止时回安全中位；
- 联调时显示有效帧数、CRC 错误数、格式错误数、控制状态和故障码。

## 2. 树莓派开工前需要的参数

下表是必须确认的参数清单。“当前值”是现有代码或已确认方案；“待实测”不能靠猜测填写。

### 2.1 TI 已确定的接口参数

| 参数 | 当前值 | 树莓派用途 |
|---|---:|---|
| `protocol_version` | `1` | 写入每个数据帧 |
| `frame_header` | `A5 5A` | 帧同步 |
| `frame_size` | `21 bytes` | 完整帧长度 |
| `baudrate` | `115200` | 串口初始化 |
| `uart_format` | `8-N-1` | 串口初始化 |
| `endianness` | little-endian | 整数、浮点和 CRC |
| `minimum_confidence` | `0.70` | TI 接受测量的当前阈值 |
| `position_min_mm` | `-120.0` | TI 当前软件硬范围 |
| `position_max_mm` | `+120.0` | TI 当前软件硬范围 |
| `dt_min_ms` | `5` | 相邻有效视觉帧的最小间隔 |
| `dt_max_ms` | `150` | 相邻有效视觉帧的最大间隔 |
| `vision_timeout_ms` | `180` | 无有效测量后的安全超时 |
| `arming_valid_frames` | `3` | 连续有效帧启动 |

这些是当前软件配置，不代表最终实物范围。若机械可滚动范围更小，应同时收紧树莓派和 TI 的限制。

### 2.2 机械组/主控组必须提供给树莓派的参数

| 参数 | 状态 | 说明 |
|---|---|---|
| `tube_usable_length_mm` | 待实测 | 钢球中心可安全运动的有效长度，不是水管外形总长 |
| `position_min_mm` | 待按实物复核 | `O` 到 `P-` 的安全距离，负值 |
| `position_max_mm` | 待按实物复核 | `O` 到 `P+` 的安全距离，正值 |
| `position_zero_mark` | 待在实物标记 | 控制零点 `O` 的准确位置 |
| `positive_end_mark` | 待在实物标记 | 明确哪一端为 `P+` |
| `negative_end_mark` | 待在实物标记 | 明确哪一端为 `P-` |
| `ball_diameter_mm` | 待用卡尺确认 | 帮助约束检测半径和毫米标定 |
| `tube_reference_markers` | 待确定 | 水管两端是否增加易识别标记 |
| `allowed_tube_angle_deg` | 待实测 | 用于保证相机 ROI 覆盖全部允许姿态 |
| `camera_mount_height_mm` | 待实测 | 用于估算球的像素尺寸和视场 |

其中 `P- / O / P+` 必须直接标在实物或标定板上，不能只在软件里口头约定“左边/右边”。

### 2.3 树莓派队友必须实测并保存的视觉参数

| 参数 | 建议配置位置 | 要求 |
|---|---|---|
| `camera_device` | `config.yaml` | 确认实际设备节点 |
| `frame_width`、`frame_height` | `config.yaml` | 固定，不允许运行中变化 |
| `requested_fps` | `config.yaml` | 建议 30–60 FPS，最低目标 25 FPS |
| `exposure` | `config.yaml` | 尽量固定，兼顾亮度和运动拖影 |
| `gain` | `config.yaml` | 尽量固定，避免噪声随时间变化 |
| `white_balance` | `config.yaml` | 彩色阈值法使用时固定 |
| `roi` | `calibration.json` | 覆盖水管所有允许倾角 |
| `pixel_p_negative` | `calibration.json` | `P-` 标记的标定位置 |
| `pixel_zero` | `calibration.json` | `O` 标记的标定位置 |
| `pixel_p_positive` | `calibration.json` | `P+` 标记的标定位置 |
| `camera_matrix` | `calibration.json` | 广角或畸变明显时必须提供 |
| `distortion_coeffs` | `calibration.json` | 与相机内参配套 |
| `ball_radius_px_min/max` | `config.yaml` | 由真实钢球和安装高度测量 |
| `ball_area_min/max` | `config.yaml` | 排除反光点和大面积干扰 |
| `circularity_min` | `config.yaml` | 排除非球形候选 |
| `position_jump_gate_mm` | `config.yaml` | 按最大真实球速和帧间隔确定 |
| `valid_confidence_target` | `config.yaml` | 稳定检测建议明显高于 `0.70` |
| `serial_port` | `config.yaml` | 推荐 `/dev/serial0`，以实际映射为准 |
| `baudrate` | `config.yaml` | 固定 `115200` |
| `log_directory` | `config.yaml` | 每次实验保存独立日志 |

## 3. 统一坐标定义

在水管上标记：

```text
P- -------- O -------- P+
```

定义：

- `O`：控制零点，`position_mm = 0.0`；
- `P+`：钢球向该端移动时，`position_mm` 增大；
- `P-`：钢球向该端移动时，`position_mm` 减小；
- 正水管倾角：重力使钢球向 `P+` 加速。

坐标必须相对水管，而不是相对整幅图像或车体。

摄像头固定在车架正上方，不随水管倾斜。设本帧检测到的球心为 `B`，两端参考点为 `P-`、`P+`，可先计算球心沿当前管轴的归一化投影：

```text
axis = P+ - P-
u = dot(B - P-, axis) / dot(axis, axis)
```

再用现场标定将 `u` 转为毫米。若 `O` 不在两端几何中点，应使用以 `O` 为中心的分段映射：

```text
P- 到 O：映射为 position_min_mm 到 0
O 到 P+：映射为 0 到 position_max_mm
```

若有镜头畸变或透视，应先去畸变，并使用单应变换或多点查表。

必须通过以下测试后才能调 PID：

1. 球放在 `O` 时输出接近 `0 mm`；
2. 球移向 `P+` 时数值单调增大；
3. 球移向 `P-` 时数值单调减小；
4. 水管小角度倾斜、球相对水管不动时，位置基本不变；
5. 在多个已知毫米位置测量误差和静态抖动。

## 4. 钢球检测和 `confidence`

检测方案按真实画面选择，可使用：

- HSV/Lab 阈值与连通域；
- 灰度阈值、边缘和圆检测；
- 背景差分；
- 模板匹配；
- 上述方法结合水管 ROI、半径、面积、圆度和运动连续性筛选。

第一版优先保证稳定、低延迟和可解释，不要求使用深度学习。

`confidence` 必须位于 `0.0~1.0`，可综合：

```text
圆度得分
半径/面积范围得分
是否位于有效水管区域
与上一帧预测位置的连续性
水管参考点是否有效
轮廓对比度
是否存在多个难以区分的候选
```

建议规则：

- 球和水管参考均清晰：目标 `confidence ≥ 0.80`；
- 候选不唯一、轮廓残缺或位置跳变：降低置信度；
- 球丢失或水管参考无效：`confidence = 0.0`；
- 不得为了让 TI 保持运行而伪造高置信度。

检测失败时仍可发送本次新图像对应的低置信度帧：

```text
position_mm = 0.0
confidence = 0.0
sequence 正常递增
capture_timestamp_us 对应本次新图像
```

TI 不会把低置信度帧用于控制；连续没有有效测量时会触发本地视觉超时并让舵机回安全位置。

## 5. UART 接线

| 树莓派 | MSPM0G3507 |
|---|---|
| TX | `B16 / UART2_RX` |
| RX | `B15 / UART2_TX` |
| GND | GND |

要求：

- TX/RX 交叉；
- 两端共地；
- 使用 3.3 V TTL 电平；
- 不接 RS-232 电平转换器；
- 不向 MSPM0 UART 输入 5 V；
- 不使用硬件流控；
- 树莓派 UART 不再作为 Linux 登录控制台。

若使用树莓派 40 针 GPIO，常见映射为：

```text
GPIO14 / TXD
GPIO15 / RXD
GND
```

程序中优先使用 `/dev/serial0`，但必须通过系统实际映射确认，不要硬编码假定一定是 `/dev/ttyAMA0`。

## 6. 21 字节视觉帧 V1

所有多字节字段均为小端；浮点使用 IEEE-754 `float32`。

| 偏移 | 大小 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | `header0` | 固定 `0xA5` |
| 1 | 1 | `header1` | 固定 `0x5A` |
| 2 | 1 | `version` | 固定 `1` |
| 3 | 4 | `sequence` | 单调递增 `uint32` |
| 7 | 4 | `capture_timestamp_us` | 采图时的 `uint32` 微秒计数 |
| 11 | 4 | `position_mm` | 相对水管的 `float32` 毫米坐标 |
| 15 | 4 | `confidence` | `float32`，范围 `0.0~1.0` |
| 19 | 2 | `crc16` | CRC 小端，低字节在前 |

CRC 使用 CRC-16/CCITT-FALSE：

```text
覆盖范围：偏移 2..18，共 17 字节
poly=0x1021
init=0xFFFF
refin=false
refout=false
xorout=0x0000
```

协议中没有目标位置字段。`+50 mm`、`-50 mm` 和 `0 mm` 目标由 TI 本地比赛状态机设置。

协议中也没有舵机角度或 PWM 字段。树莓派只报告测量。

## 7. 协议测试向量

输入：

```text
sequence=1
capture_timestamp_us=1000000
position_mm=0.0
confidence=1.0
```

期望 CRC：

```text
0xCC99
```

期望完整 21 字节：

```text
A5 5A 01 01 00 00 00 40 42 0F 00 00 00 00 00 00 00 80 3F 99 CC
```

树莓派协议单元测试必须逐字节得到完全相同的结果。

CRC 的标准校验向量还应满足：

```text
CRC-16/CCITT-FALSE("123456789") = 0x29B1
```

## 8. Python 打包参考

```python
from __future__ import annotations

import math
import struct


HEADER = b"\xA5\x5A"
PROTOCOL_VERSION = 1
FRAME_SIZE = 21


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_vision_frame(
    sequence: int,
    capture_timestamp_us: int,
    position_mm: float,
    confidence: float,
) -> bytes:
    if not math.isfinite(position_mm):
        raise ValueError("position_mm must be finite")
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be finite and in [0, 1]")

    payload = struct.pack(
        "<BIIff",
        PROTOCOL_VERSION,
        sequence & 0xFFFFFFFF,
        capture_timestamp_us & 0xFFFFFFFF,
        position_mm,
        confidence,
    )
    crc = crc16_ccitt_false(payload)
    frame = HEADER + payload + struct.pack("<H", crc)
    assert len(frame) == FRAME_SIZE
    return frame
```

测试：

```python
frame = make_vision_frame(
    sequence=1,
    capture_timestamp_us=1_000_000,
    position_mm=0.0,
    confidence=1.0,
)

assert frame.hex(" ").upper() == (
    "A5 5A 01 01 00 00 00 40 42 0F 00 "
    "00 00 00 00 00 00 80 3F 99 CC"
)
```

## 9. 发送循环要求

参考框架：

```python
import time
import serial


uart = serial.Serial(
    port="/dev/serial0",
    baudrate=115200,
    bytesize=8,
    parity=serial.PARITY_NONE,
    stopbits=1,
    timeout=0,
    write_timeout=0.05,
)

sequence = 0

while True:
    image = camera.read()
    capture_timestamp_us = time.monotonic_ns() // 1000

    result = detect_ball_relative_to_tube(image)
    if result.valid:
        position_mm = result.position_mm
        confidence = result.confidence
    else:
        position_mm = 0.0
        confidence = 0.0

    frame = make_vision_frame(
        sequence,
        capture_timestamp_us,
        position_mm,
        confidence,
    )
    uart.write(frame)
    sequence = (sequence + 1) & 0xFFFFFFFF
```

实际项目中，时间戳应尽量靠近相机真正完成采集的时刻。若相机 API 能返回硬件时间戳，优先使用；否则在读取帧的最邻近位置调用单调时钟。

发送规则：

- 每个新图像结果只发一次；
- 不为提高表面帧率重复发送旧位置；
- `sequence` 每发送一帧递增；
- 允许 `uint32` 自然回卷；
- 只有一个线程写 UART；
- 禁止把 CSV 调试文本混入二进制 UART；
- 调试日志写文件或使用另一条接口。

## 10. 推荐树莓派工程结构

```text
rpi_ball_vision/
├─ main.py
├─ config.yaml
├─ camera_source.py
├─ tube_reference.py
├─ ball_detector.py
├─ calibration.py
├─ vision_result.py
├─ ti_protocol.py
├─ ti_serial_link.py
├─ debug_view.py
├─ requirements.txt
├─ calibration.json
├─ tests/
│  ├─ test_protocol.py
│  ├─ test_calibration.py
│  └─ test_recorded_frames.py
└─ logs/
```

推荐数据结构：

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class VisionResult:
    sequence: int
    capture_timestamp_us: int
    position_mm: float
    confidence: float
    ball_center_px: tuple[float, float] | None
    tube_reference_valid: bool
    process_time_ms: float
```

推荐模块调用：

```text
camera_source.read()
  → tube_reference.find(frame)
  → ball_detector.detect(frame, tube_reference)
  → calibration.to_position_mm(...)
  → VisionResult
  → ti_protocol.make_vision_frame(...)
  → ti_serial_link.write(...)
```

检测、标定、协议、串口和调试显示不能分别维护不同版本的位置变量。

## 11. TI 代码的对应关系

| 树莓派输出/事件 | TI 文件 | TI 动作 |
|---|---|---|
| UART 字节流 | `ball_vision_link.c/.h` | 帧头、长度、版本、CRC、浮点范围校验 |
| 完整视觉帧 | `task3_ball.c/.h` | 通过 `task3_ball_submit_measurement()` 提交 |
| `position_mm`、`confidence`、时间戳 | `ball_control.c/.h` | 位置滤波、速度估计和串级控制 |
| 目标水管角 | `ball_actuator.c/.h` | 三点映射为 B4 脉宽并限幅 |
| 舵机脉宽 | `servo.c/.h` | B4 PWM 输出 |
| 无有效视觉 | `ball_control.c/.h` | 180 ms 超时、故障锁存和安全回中 |

当前 Task3 状态机：

```text
READY
  → WAIT_SENSOR
  → TO_PLUS_50
  → TO_MINUS_50
  → DONE_HOLD
```

树莓派不需要知道当前目标点，只需持续提供测量。

## 12. 当前 TI 接入状态

目前 TI 已具备：

- `ball_vision_link` 二进制解析器；
- `ball_control` 位置—速度串级控制；
- `ball_actuator` 水管角到舵机脉宽映射；
- `task3_ball` 比赛状态机；
- 5 ms Task3 和舵机服务。

当前缺少的关键连接：

> `UART2_IRQHandler()` 尚未把 Task3 收到的字节交给 `ball_vision_link`；非电脑 PID 调参任务的 UART2 数据目前会被清空。

电脑 PID 调参任务使用 ASCII 协议，滚球视觉使用二进制协议。TI 端必须按任务互斥：

```text
电脑调参任务 → ASCII 解析器
Task3 滚球任务 → 21 字节视觉解析器
其他任务 → 丢弃或按各自明确协议处理
```

推荐 TI 接入边界：

```c
/* UART2 ISR：只做逐字节解析。 */
ball_vision_link_rx_byte_from_isr(&vision_link, rx_byte, now_us);

/* 5 ms 普通控制上下文：读取完整快照并提交。 */
if (ball_vision_link_take_latest(&vision_link, &frame)) {
    task3_ball_submit_measurement(&frame.measurement);
}
```

不要在 UART ISR 中运行 PID、舵机控制、字符串格式化、OLED 刷屏或任何阻塞操作。

正式联调前，TI 端应临时显示或通过调试器观察：

```text
valid_frame_count
crc_error_count
format_error_count
last sequence
last position_mm
last confidence
Task3 stage
ball_control fault
```

树莓派 V1 目前是单向测量协议，没有必须等待的 TI 应答。联合验帧依靠 TI 端计数器、OLED/调试器和双方日志。

## 13. 舵机和连杆参数的边界

树莓派不需要短杆、长杆长度来计算控制输出。TI 的 `ball_actuator` 使用实测三点映射：

```text
水管负角 → pulse_at_min_angle_us
水管水平 → pulse_neutral_us
水管正角 → pulse_at_max_angle_us
```

当前软件默认：

```text
-5° → 1350 µs
 0° → 1500 µs
+5° → 1650 µs
安全脉宽 → 1500 µs
硬脉宽范围 → 1100~1900 µs
```

这些只是待实测的安全初值。机械组和 TI 端必须确认：

- 舵机短杆接近 90°时，水管是否真的水平；
- 水平脉宽；
- 正负安全最大水管角；
- 每个安全角对应的脉宽；
- 正角是否让球向 `P+` 加速；
- 连杆是否碰撞、过中心或进入死区；
- 同一脉宽往返后的回差。

这些参数只影响 TI 角度—脉宽映射，不改变树莓派的 21 字节视觉协议。

## 14. 日志要求

树莓派 CSV 至少包含：

```text
host_time
sequence
capture_timestamp_us
ball_pixel_x
ball_pixel_y
tube_p_negative_x
tube_p_negative_y
tube_p_positive_x
tube_p_positive_y
tube_reference_valid
position_mm
confidence
frame_period_ms
process_time_ms
uart_write_time_ms
```

调试画面至少叠加：

- 水管 ROI；
- `P- / O / P+`；
- 当前水管轴线；
- 钢球中心和候选半径；
- `position_mm`；
- `confidence`；
- FPS 和处理耗时；
- 检测有效/失效状态。

## 15. 联调顺序

### 阶段 A：离线视觉

1. 不接 TI 和舵机；
2. 连续运行至少 10 分钟；
3. 验证 `P- / O / P+` 和多个中间已知位置；
4. 记录毫米误差和静态抖动；
5. 验证水管运动但球相对水管不动时的位置稳定性；
6. 遮挡球和水管标记，确认 `confidence` 降为无效；
7. 保存原图、叠加视频和 CSV。

### 阶段 B：协议验帧

1. 不连接舵机机构；
2. 用本文测试向量验证树莓派打包；
3. 核对实际串口为 115200-8-N-1；
4. 在 TI 端观察有效帧、CRC 错误和格式错误计数；
5. 检查 `position_mm`、`confidence`、序号和时间戳；
6. 停止树莓派发送，确认 TI 超时；
7. 重新发送连续有效帧，确认启动逻辑符合预期。

### 阶段 C：舵机开环安全标定

1. 不放钢球；
2. 确认舵机水平中位；
3. 小幅测试正负方向；
4. 测量实际水管角和对应脉宽；
5. 确认机械无碰撞、无过中心；
6. 填写 `ball_actuator` 实测参数；
7. 失联时确认回到安全中位。

### 阶段 D：低增益闭环

1. 目标先固定 `0 mm`；
2. `Ki_pos=0`、`Kd_vel=0`、加速度前馈为 `0`；
3. 先验证水管动作和钢球加速度方向；
4. 从很小的角度限幅和速度限幅开始；
5. 调速度环比例；
6. 再调位置环比例和目标速度上限；
7. 中心稳定后测试 `+50 mm → -50 mm`；
8. 最后评估是否需要积分、微分或角度传感器。

## 16. 树莓派交付验收

交付给 TI 联调前必须具备：

- 可直接运行的完整源码；
- `requirements.txt`；
- `config.yaml`；
- 实机 `calibration.json`；
- 相机和串口启动命令；
- 21 字节协议单元测试；
- CRC 两个测试向量；
- 10 分钟稳定性日志；
- 多点毫米标定报告；
- 遮挡和失视测试记录；
- 一段带检测叠加的真实视频；
- 一份真实 CSV。

树莓派部分完成的判定标准：

```text
钢球真实位置
  → 稳定识别钢球和本帧水管参考
  → 输出方向正确、单位正确的 position_mm
  → 正确计算 confidence 和采图时间戳
  → 每张新图只产生一个 21 字节帧
  → TI 的 valid_frame_count 连续增加
  → CRC/格式错误计数不持续增加
  → 遮挡和断线时 TI 进入安全状态
```

## 17. 常见错误

### TI 一直停在 `WAIT_SENSOR`

依次检查：

1. TI UART2 是否真的已接到 `ball_vision_link`；
2. 当前任务是否为 Task3；
3. 波特率是否为 115200；
4. TX/RX 是否交叉且共地；
5. 帧是否严格 21 字节；
6. CRC 是否覆盖偏移 `2..18`；
7. CRC 是否小端发送；
8. `confidence` 是否达到 `0.70`；
9. `position_mm` 是否位于当前有效范围；
10. 序号和时间戳是否变化。

### TI 有效帧计数增加但很快超时

- 实际有效 FPS 太低；
- 视觉处理偶发阻塞超过 180 ms；
- 检测帧置信度经常低于阈值；
- 重复发送旧序号；
- 时间戳间隔异常；
- 串口写入线程被调试显示或磁盘写入阻塞。

### 钢球越控越远

先关闭闭环，检查：

- `P+` 定义是否一致；
- 球向 `P+` 运动时 `position_mm` 是否增加；
- TI 正水管角是否使球向 `P+` 加速；
- 舵机正负脉宽标定是否反向。

这是方向错误，不能通过降低或提高 PID 增益修复。

### 静止球的位置随水管倾斜明显变化

- 水管端点/参考标记没有每帧更新；
- 直接使用了固定画面 `pixel_x`；
- ROI 或轴线检测错误；
- 镜头畸变未校正；
- 标记被舵机或连杆遮挡。

解决几何测量后再调控制器。
