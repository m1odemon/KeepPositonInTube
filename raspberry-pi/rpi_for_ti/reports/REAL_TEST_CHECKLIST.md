# 实机视觉与 TI 联调记录

> 这份文件只提供模板。所有结果必须由真实水管、钢球、相机和 TI 板实测填写。

## 基本信息

- 日期：
- 操作者：
- 相机型号与 `/dev/video*`：
- 相机安装高度：
- 水管有效长度：
- 钢球直径：
- Git 提交：
- `config.yaml` 备份：
- `calibration.json` 备份：

## 相机固定参数

- 分辨率/格式/FPS：
- 曝光：
- 增益：
- 白平衡：
- 实测采集 FPS：
- 视觉主循环 FPS：
- 平均/最大处理耗时：

## 多点毫米标定

填写同目录的 `multipoint_calibration.csv`，至少覆盖：

```text
P-、-50 mm、O、+50 mm、P+
```

记录平均误差、最大绝对误差和静态标准差。

## 失效测试

| 测试 | 预期 | 实测 |
|---|---|---|
| 遮挡钢球 | confidence=0，三帧内 LOST | |
| 遮挡红色 P- | position=0，confidence=0 | |
| 遮挡蓝色 P+ | position=0，confidence=0 | |
| 移除摄像头 | 程序报错，TI 180 ms 超时回安全位 | |
| 停止 UART | TI 180 ms 超时回安全位 | |
| 水管倾斜、球相对管不动 | 位置变化在允许误差内 | |

## 10 分钟稳定性

- CSV 文件：
- 调试视频：
- 总帧数：
- 有效帧比例：
- 最低/平均 confidence：
- 相机读取失败数：
- TI valid frame：
- TI CRC error：
- TI format error：
- 结论：

## 闭环前签字

- [ ] P- / O / P+ 方向一致
- [ ] 毫米标定已写入 `calibration.json`
- [ ] 真实位置误差满足任务要求
- [ ] confidence 失效逻辑验证
- [ ] TI 超时安全回中验证
- [ ] 舵机方向和机械限位验证
- [ ] 先低增益测试 0 mm，再测试 ±50 mm
