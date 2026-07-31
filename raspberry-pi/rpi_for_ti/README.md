# 树莓派钢球视觉 → TI MSPM0G3507

这是按 [`README_BALL_RPI_TEAM.md`](README_BALL_RPI_TEAM.md) 实现的树莓派端工程。视觉算法使用此前调好的“第二套方案 Blob”，但不再依赖固定 ROI：每一帧先检测水管两端参考标记，再把倾斜水管透视变换为水平条带，在条带内进行灰度二值化、连通域筛选和时间跟踪。

## 1. 实物标记和坐标

必须在随水管一起运动的位置贴两个哑光标记：

```text
红色 P- -------- O -------- 蓝色 P+
```

- 红色中心定义为负端 `P-`；
- 蓝色中心定义为正端 `P+`；
- 球向蓝色端运动时 `position_mm` 必须增大；
- 标记不能贴在球会遮挡的位置，建议贴到两端管托或延伸片上；
- 使用哑光贴纸，避免高光导致 HSV 标记断裂。

算法每帧检测两个标记，建立本帧水管轴线并生成动态 ROI，所以相机轻微晃动、水管在画面内倾斜时，输出仍是球相对水管的位置。任一标记丢失时，本帧发送：

```text
position_mm = 0.0
confidence = 0.0
```

## 2. 当前 Blob 参数

`config.yaml` 已保存换管后的实测配置：

| 项目 | 当前值 |
|---|---:|
| 面积 | `1400～4000`，期望 `2910` |
| 宽度 | `22～52 px`，期望 `36 px` |
| 高度 | `65～115 px`，期望 `97 px` |
| 宽高比 | `0.24～0.65`，期望 `0.37` |
| 最小候选得分 | `0.40` |
| 三帧确认 | `3` |
| 丢失判定 | `3` 帧 |
| 跳变门限 | `30 mm` |
| TI 接受置信度 | `0.70` |

检测链路为：

```text
640×480 MJPEG 120 FPS
  → 红/蓝参考标记
  → 每帧水管轴线和透视条带
  → 灰度 + 高斯滤波 + Otsu 反色二值化
  → 闭运算连接钢球高光造成的断裂
  → connectedComponentsWithStats
  → 面积/宽高/宽高比/圆度/实心度/填充率筛选
  → 候选歧义降置信度
  → 三帧确认、位置跳变门限和指数平滑
  → 毫米坐标、confidence
```

注意：Blob 参数是在真实旧画面像素尺度下得到的。动态条带会随红蓝标记间距缩放。如果实机叠加画面中的 `A/W/H` 明显偏离表格，应固定相机安装和标记位置后重新测量，而不是盲目放宽全部范围。

## 3. 工程结构

```text
rpi_for_ti/
├─ main.py                         # 便捷入口
├─ config.yaml                     # 相机、参考标记、Blob、串口参数
├─ calibration.json                # 实机毫米标定，当前数值仍需复核
├─ requirements.txt
├─ pyproject.toml                  # 可编辑安装及 hball-ti 命令
├─ RUN_TI_COMMANDS.txt             # 树莓派完整命令集
├─ README_BALL_RPI_TEAM.md         # 原始接口要求
├─ src/hball_ti/
│  ├─ camera_source.py             # GStreamer + 最新帧采集线程
│  ├─ tube_reference.py            # 红/蓝端点、动态轴线、透视 ROI
│  ├─ ball_detector.py             # 第二套 Blob + 时间跟踪
│  ├─ travel_timer.py              # 起终点门线、采集时间戳计时
│  ├─ calibration.py               # 归一化轴坐标 → 毫米
│  ├─ vision_pipeline.py           # 单一视觉数据链
│  ├─ vision_result.py             # 唯一结果数据结构
│  ├─ ti_protocol.py               # 21 字节 V1 与 CRC
│  ├─ ti_serial_link.py            # 115200、8-N-1 单写线程
│  ├─ session_logger.py            # 联调 CSV
│  ├─ debug_view.py                # 测试叠加画面/视频
│  └─ main.py                      # 采集、识别、发送主循环
├─ tools/
│  ├─ self_check.py                # 环境、配置和固定向量自检
│  ├─ send_test_vector.py          # 不开相机，单独验 TI 串口
│  ├─ setup_dynamic_roi.py         # 新相机高度拍照和动态 ROI 设置
│  └─ travel_time_test.py          # 第二套 Blob 行程计时程序
├─ tests/                          # 协议、标定、动态视觉单元测试
└─ reports/                        # 实机验收模板
```

采集线程只保留最新帧，推理/显示慢时不会积压旧图。每张实际处理的新图只发送一个 21 字节帧，串口只由主线程写入。

## 4. 首次安装

项目位于 `~/yolo_usb/rpi_for_ti`，复用父目录的 `.venv`：

```bash
cd ~/yolo_usb/rpi_for_ti
source ../.venv/bin/activate
python -m pip install -e .
```

先确认当前 OpenCV 带 GStreamer：

```bash
python -c "import cv2; print([x.strip() for x in cv2.getBuildInformation().splitlines() if 'GStreamer:' in x])"
```

应显示 `GStreamer: YES`。完整命令和排错见 [`RUN_TI_COMMANDS.txt`](RUN_TI_COMMANDS.txt)。

## 5. 运行模式

离线调视觉（不接 TI，有窗口、CSV、调试视频）：

```bash
hball-ti --dry-run --show --record-video videos/blob_tune.mp4
```

接 TI 联调（有窗口和 CSV）：

```bash
hball-ti --show
```

比赛模式（无窗口、无本地视频、无 CSV，把性能留给识别和 UART）：

```bash
hball-ti --no-csv
```

窗口按 `Q` 或 `Esc` 退出。默认不保存录像；只有显式给出 `--record-video` 才会写视频。

## 6. 标定必须实测

`calibration.json` 中的 `-100 / 0 / +100 mm` 只是占位初值，不能直接用于闭环比赛。固定相机和标记后：

1. 用卡尺量出钢球中心从 `P-` 到 `O`、从 `O` 到 `P+` 的安全距离；
2. 将负端距离写成 `position_min_mm`，正端距离写成 `position_max_mm`；
3. `zero_u` 是零点在红到蓝轴线上的比例，中心点才是 `0.5`；
4. 球放在 `O`，确认输出接近 `0 mm`；
5. 向蓝端移动应单调增大，向红端移动应单调减小；
6. 在至少 5 个已知位置记录误差和静态抖动；
7. 把结果填入 `reports/multipoint_calibration.csv`。

广角畸变明显时，应把 OpenCV 标定得到的 `camera_matrix` 和 `distortion_coeffs` 写入 JSON。

## 7. confidence 和 TI

最终置信度由参考标记质量和 Blob 质量加权得到。球或参考标记丢失时为零；多候选、几何偏差和位置跳变会降低置信度。TI 当前只接受 `confidence >= 0.70`，但树莓派仍会发送每个新图像对应的低置信度帧，以维持正确的序号和时间戳语义。

协议固定为：

```text
A5 5A | version=01 | sequence uint32 | timestamp uint32
      | position float32 | confidence float32 | CRC16 little-endian
```

固定测试向量：

```text
A5 5A 01 01 00 00 00 40 42 0F 00 00 00 00 00 00 00 80 3F 99 CC
```

## 8. 现场验收

代码完成不等于实机标定完成。交给 TI 闭环前，必须完成：

- 10 分钟连续运行；
- 5 点以上毫米标定；
- 静止球抖动统计；
- 水管倾斜但球相对水管不动的稳定性测试；
- 遮挡球、遮挡红标、遮挡蓝标测试；
- TI 有效帧、CRC 错误、格式错误和 180 ms 超时测试；
- 保存一份真实 CSV 和一段测试叠加视频。

使用 [`reports/REAL_TEST_CHECKLIST.md`](reports/REAL_TEST_CHECKLIST.md) 逐项记录，不要伪造尚未实测的数据。

## 9. 钢球行程时间测试

独立计时脚本复用同一套动态管道参考和 Blob 检测，不连接 TI。示例：测量钢球从 `-50 mm` 到 `+50 mm`，重复 3 次：

```bash
python tools/travel_time_test.py \
  --start-mm -50 \
  --end-mm 50 \
  --tolerance-mm 5 \
  --stable-frames 5 \
  --runs 3
```

操作过程：

1. 把球放入起点黄色 `START` 区域并保持；
2. 连续 5 个有效帧后画面显示 `READY`；
3. 球沿终点方向离开起点区域后显示 `TIMING`；
4. 球进入紫色 `END` 区域后输出时间、有效测量距离和平均速度；
5. 多次测试时，把球重新放回起点，等待再次 `READY`。

默认容差为 `5 mm`。因此 `-50 → +50 mm` 实际计时门线是 `-45 → +45 mm`，有效距离为 `90 mm`。这种定义能避免静止球在指定点附近抖动反复触发。需要更接近指定坐标时，可改为 `--tolerance-mm 2`，但不能小于实际静态抖动。

脚本使用 Blob 候选的原始中心位置计时，不使用显示窗口时间，也不使用平滑后位置；起止跨线时会在相邻两张图之间线性插值。结果保存到：

```text
logs/travel_time_日期_时间.csv
```

按 `R` 可放弃当前一次并重新等待起点，按 `Q` 或 `Esc` 退出。无窗口测试使用：

```bash
python tools/travel_time_test.py \
  --start-mm -50 \
  --end-mm 50 \
  --runs 10 \
  --no-show
```

摄像头高度改变后，必须先运行一次 `hball-ti --dry-run --show --no-csv`，检查新画面中的 `A/W/H`。如果明显不再接近 `2910/36/97`，应按新高度收集稳定值并修改 `config.yaml`，否则计时脚本会因为 Blob 几何筛选失败而不启动。

## 10. 摄像头高度改变后重新设置动态 ROI

本项目不能使用固定矩形 ROI，因为比赛时水管会相对车载摄像头倾斜。重新安装相机后运行：

```bash
python tools/setup_dynamic_roi.py
```

先确保红色 `P-` 和蓝色 `P+` 都能稳定识别。黄色四边形是当前动态 ROI：

- `[`：每次把 ROI 半宽减小 `2 px`；
- `]`：每次把 ROI 半宽增加 `2 px`；
- `C`：保存当前原图、矫正后 ROI 和测量 JSON，不修改配置；
- `P`：保存照片，并把新 ROI 半宽和新管轴像素长度写入 `config.yaml`；
- `Q` / `Esc`：不保存配置并退出。

建议拍两组：

1. 空管状态按 `C`；
2. 放入钢球、保证黄色 ROI 覆盖完整球体后按 `C`；
3. 保持红蓝标记稳定至少 20 帧后按 `P` 写入配置。

照片保存在 `calibration_images/日期_时间/`。写配置前的版本自动备份为：

```text
config.before_roi_setup.yaml
```

ROI 保存后再运行：

```bash
hball-ti --dry-run --show --no-csv
```

检查新高度下钢球的 `A/W/H`。设置 ROI 只解决管道区域，不能自动保证旧 Blob 尺寸仍适用；相机高度变化较大时需要按新观测值修改 `ball_blob` 的面积、宽度、高度及期望值。
