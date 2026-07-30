# H题滚球控制——第一阶段安全基线

## 当前固件状态

- 构建目标为 `HBall`，旧双轴云台、旧裸串口和IMU姿态任务不参与链接。
- QD4310固定使用CAN ID `1`，CAN命令/反馈ID分别为 `0x401/0x501`。
- F407上电只发送零电流和禁用命令，不回绝对角零位。
- `App/BallBeamConfig.h` 中 `calibration_valid=false` 是硬安全锁；在完成七点机构标定前不得修改。
- USART6为 `PG14(TX)/PG9(RX)`、`1152000-8-N-1`。
- CAN1为 `PD1(TX)/PD0(RX)`、经典CAN 1 Mbit/s，需要CAN收发器和共地。

## 构建

```powershell
cmake -S . -B build-h -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build-h -j
```

输出文件：

- `build-h/HBall.elf`
- `build-h/HBall.bin`
- `build-h/HBall.hex`

## 第一次烧录验收

1. 使用限流电源，先不安装连杆或保证连杆可自由运动。
2. QD4310设为ID 1并接好CAN收发器、120欧终端和共地。
3. 烧录 `HBall.hex` 后上电，电机不得主动转动或保持某个绝对角度。
4. 树莓派或电脑串口连接F407后运行：

   ```powershell
   python RPI/monitor_stage1.py COM8
   ```

5. 正常的第一阶段输出应满足：
   - `state=CALIBRATION_REQUIRED`
   - `fault`包含`CALIBRATION_INVALID`
   - 电机通电并正确反馈时，`motor_feedback=YES`
   - `motor_enabled=NO`
   - 手动缓慢转动空载电机时，`angle`连续变化，`speed`正负方向合理

若没有电机反馈，先检查ID、CANH/CANL、终端电阻、收发器供电和波特率；不要解除标定安全锁。

## Pi→F407命令帧 V1

固定32字节，小端，CRC-16/CCITT-FALSE覆盖偏移 `2..29`：

| 偏移 | 字段 |
|---:|---|
| 0..1 | `A5 5A` |
| 2 | 版本 `1` |
| 3 | 类型 `0x10` |
| 4..7 | `sequence: uint32` |
| 8..11 | `capture_timestamp_us: uint32` |
| 12..15 | `ball_position_mm: float32` |
| 16..19 | `confidence: float32` |
| 20..23 | `target_position_mm: float32` |
| 24..27 | `chassis_acceleration_mps2: float32` |
| 28 | `task_id` |
| 29 | 标志：run、急停、目标有效、加速度有效 |
| 30..31 | CRC16，小端 |

F407→Pi遥测固定54字节，完整编解码定义位于 `RPI/hball_protocol.py`。
