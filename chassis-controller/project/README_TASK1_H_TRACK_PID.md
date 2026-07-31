# Task1 H题环线与 Task10 PID 调参说明

## 当前分工

- Task1：H题椭圆环线持续循迹，使用趋势前瞻、曲率保持、动态降速和自适应差速。
- Task9：保留为与 Task1 相同底盘算法的增强环线测试入口。
- Task10：使用基础误差整形持续环线，并通过 UART2 接受电脑 PID 命令和发送 CSV。

三个持续环线任务均由 `KEY4` 启动或停止，不自动计算圈数，也不识别 A 点停车线。
需要自动停回 A 点时使用 Task2。

## 已删除的旧 Task1 内容

- 六个靶点的选择页面和目标数量校验。
- 丰字赛道主干/支路坐标与方向规划。
- 十字路口识别及前探状态。
- 90° 原地转向、180° 调头和转弯恢复状态。
- `cross_state`、`target_selected[]`、`current_node` 等相关接口。

## Task10 UART2 调参

UART2 参数保持 `115200 baud`、TX=`B15`、RX=`B16`。只有选中 Task10 时才解析：

```text
STATUS
SET P:7.0 I:0 D:0.2
```

主循环同时发送：

```text
timestamp,setpoint,input,pwm,error,Kp,Ki,Kd
```

Task10 修改的是全局 `PIDK_YA`，因此调好后切换到 Task1、Task2 或 Task9，
新的外环 PID 参数仍然生效。但 Task1/Task2/Task9 还叠加了增强误差整形，
同一组数值仍需低速复测，不能认为 Task10 调好后无需验证。树莓派任务二进制
协议继续保持断开。

## 实车验证顺序

1. 架空车轮，确认 Task1 和 Task10 均可由 `KEY4` 立即停止。
2. Task10 连接电脑，先发送 `STATUS`，再用小步长修改 PID。
3. 低速运行 Task10，确认 CSV 与实际偏差方向一致。
4. 切换 Task1，验证增强弧线算法和新的 PID 参数能够共同工作。
