# H 题循迹停止与时间显示：Codex 对话交接

> 创建时间：2026-07-30  
> 工作区：`D:\Codex_TI_NEW\工程模板\SeekFree_common`  
> 用途：供其他 Codex 窗口接手本对话。开始修改前必须先读本文档，并重新执行 `git status --short --branch`。

## 1. 当前持续任务

维护“车载平衡滚球运动控制系统（H题）”项目的循迹停止与行驶时间显示功能。

协作规则：

1. 严格按照用户给出的步骤和双方讨论逐步修改，不自行一次性接入全部功能。
2. 用户要求“先写成函数”时，只增加独立函数，不接入主函数、中断、按键状态机或现有 UI。
3. 每次接入前重新向用户确认要选用的策略、参数和调用位置。
4. 功能修改使用 `codex/` 前缀的新分支；验证后写清楚提交说明，再合并回 `master`。
5. 修改前检查工作区，保留其他窗口和用户已有的未提交/已暂存内容，禁止整体回退或清理。
6. 未经实车 ADC、Yaw 和停车位置测试，不宣称已经满足停车误差不超过 2 cm。

赛题 PDF：

```text
C:\Users\18371\Downloads\车载平衡滚球运动控制系统（H题）.pdf
```

## 2. 本次对话已经确定的设计

赛题要求小车沿 H 题路线运行一圈后停车，停车位置距离原位置不超过 2 cm。

讨论过三种停车策略：

1. `h_stop_by_yaw_355()`：累计 Yaw 变化达到 355° 后直接停车。
2. `h_stop_by_black_line(continue_ms)`：识别 A 点黑色横线后立即停车，或继续运行指定时间后停车。
3. `h_stop_by_yaw_slow_black_line(slow_yaw_deg, slow_speed, continue_ms)`：累计 Yaw 达到 300°/330° 等阈值后降速，再检测 A 点黑线并立即停车或延时停车。

总体判断：第三种“Yaw 负责末段布防和降速、黑线负责位置闭环”更适合作为最终方案；但 ADC 经过黑线的数据目前尚未完成实车测试，阈值和延时必须后续标定。

## 3. 当前代码的真实接入状态

### 3.1 三个停车策略函数

声明：

```text
project/code/tracking_control.h
```

实现：

```text
project/code/tracking_control.c
```

当前 `master` 中：

- `h_stop_by_yaw_355()`：保留为独立备选函数，没有被任务流程直接调用。
- `h_stop_by_black_line()`：保留为独立备选函数，没有被任务流程直接调用。
- `h_stop_by_yaw_slow_black_line()`：已经被 `h_task2_update_5ms()` 调用，Task2 当前使用第三种组合策略。

因此，不能再笼统地说“三个停车函数全部没有引用”。准确说法是：前两个仍为备用函数，第三个已经接入 Task2。

2026-07-30 实车观察补充：A 点横向黑线主要由 CH3、CH4、CH5 识别。已在
独立分支完成“中心三路归一化值求和判断”：

```text
branch: codex/h-stop-ch345-sum
commit: 774c6b4 fix: detect H start line with CH3-CH5 sum
```

该提交使用 `CH3+CH4+CH5 >= 240` 作为待标定初值，保留 300° 后布防、
连续 15 ms 确认和 `StopDly`，并在 Task2 OLED 页面显示当前 `CH345`
和值和阈值。Keil Rebuild 为 0 error / 0 warning。

该提交已使用 Git `--autostash` 安全快进合并到 `master`，其他窗口的未提交
修改已恢复，原先已暂存的 `project/README_BALL_RPI_TEAM.md` 也恢复为已暂存
状态。合并后 Keil Rebuild 结果：

```text
Program Size: Code=60376 RO-data=4772 RW-data=404 ZI-data=6740
0 Error(s), 0 Warning(s)
```

相关提交：

```text
8a0051b feat: replace Task2 with H lap stop mode
```

### 3.2 独立行驶时间显示函数

声明：

```text
project/code/display.h
```

实现：

```text
project/code/display.c
```

函数列表：

```c
h_drive_timer_reset()
h_drive_timer_start()
h_drive_timer_update_5ms()
h_drive_timer_stop()
h_drive_timer_get_ms()
h_drive_timer_get_state()
h_drive_timer_show()
h_drive_timer_show_state()
```

这些 `h_drive_timer_*` 函数目前仍是独立函数：

- 没有接入 `main.c`。
- 没有接入 5 ms 中断。
- 没有接入按键启动/停止状态机。
- 没有接入现有 `show_ui()`。
- `h_drive_timer_show()` 内部调用 `h_drive_timer_get_ms()` 只是模块内部调用，不代表已经接入运行流程。

因此它们目前不会自动开始计时、更新计时或显示。

相关分支和提交：

```text
codex/h-drive-time-display
c44155b feat: add standalone H-track drive timer display
c88185b merge: add standalone H-track drive timer display
```

### 3.3 注意：Task2 已有另一套计时

当前 Task2 已经使用：

```c
h_task2_elapsed_ms
h_task2_run_finished
h_task2_prepare_start()
```

这套 `h_task2_*` 计时与备用的 `h_drive_timer_*` 模块是两套不同实现。后续接入时间显示前，必须先和用户讨论：

- 继续使用 Task2 已接入的 `h_task2_*`；
- 或改为复用通用的 `h_drive_timer_*`；
- 不应让两套计时在同一任务中重复运行。

2026-07-30 用户已确认：**按照 `master` 当前实现继续，Task2 保留
`h_task2_*` 计时链路。** 因此后续不得把 `h_drive_timer_*` 再接入 Task2；
这些通用函数继续作为未引用的备用接口保留，除非用户以后明确要求删除或改作
其他任务使用。

`master` 当前已经具备的完整链路：

```text
Task2 按键发车
  -> h_task2_prepare_start() 清零时间和完成标志
  -> TIMG6 5 ms 底盘控制周期调用 tracking_control_loop()
  -> h_task2_update_5ms() 累加 h_task2_elapsed_ms
  -> 组合停车策略成立后 h_task2_run_finished = 1
  -> 停车后不再累加，show_ui() 保留并显示最终用时
```

按上述决定保留当前实现后，已于 2026-07-30 对当前 Keil 工程执行完整
Rebuild，结果为：

```text
Program Size: Code=60532 RO-data=4768 RW-data=412 ZI-data=6748
0 Error(s), 0 Warning(s)
```

这证明当前软件能够完整编译，但不等同于已经通过实车计时精度、A 点黑线阈值
和停车位置误差测试。

另已复核 300° 后减速的调用顺序：`h_task2_update_5ms()` 先把
`speed_set_duty` 设为 20，后续 `curve_tracking_control()` 只更新循迹误差、
弯道严重程度和速度倍率，不会把 `speed_set_duty` 写回 40；
`motor_control()` 最后使用 `speed_set_duty * curve_tracking_speed_factor`
形成目标基础速度。因此当前末段减速不会在同一个 5 ms 周期内被循迹算法覆盖。
左右轮目标还有每周期最大 1.0 的斜率限制，从 40 降至 20 约需 100 ms，
属于平滑减速而非瞬间制动。

### 3.4 `h_drive_timer_*` 接入前静态审查

2026-07-30 已对独立计时模块做静态复核，未改变代码行为，结论如下：

- `start()` 先清零再进入 RUNNING，符合每次按键发车从 0 开始计时的要求。
- `reset()` 先退出 RUNNING 再清零，可避免未来接入中断后继续累加。
- `update_5ms()` 仅在 RUNNING 状态累加，并在 `UINT32_MAX` 饱和，不会回绕。
- `stop()` 保留最后时间；停止后调用显示函数仍可显示成绩。
- OLED 文本 `Time:000.000s` 使用固定宽度整数拼接，不依赖浮点格式化；显示上限
  为 999.999 s，但内部计时仍可继续超过该值。
- `row >= 8` 时不绘制，符合当前 8 行 OLED 页面边界。
- OLED 绘制只能放在主循环的 `show_ui()` 路径，不能放入 5 ms 中断。

当前 Keil 工程实际编译的是：

```text
project/user/src/main_pit_g6.c
project/user/src/isr_pit_g6.c
```

包装入口把源码中的底盘 `PIT_TIM_A1` 映射到 `PIT_TIM_G6`，真正执行
`get_ICM_data()` 和 `tracking_control_loop()` 的 5 ms 中断是
`TIMG6_IRQHandler()`。以后若接入通用计时，必须基于这个实际编译链路判断，
不能只看未直接编译的原始 `isr.c` 中函数名称。

若最终决定用 `h_drive_timer_*` 替换 Task2 现有计时，建议接入关系是：

```text
按键确认发车 -> h_drive_timer_start()
TIMG6 的 5 ms 控制周期 -> h_drive_timer_update_5ms()
实际停车成立/人工停止/安全停止 -> h_drive_timer_stop()
主循环 show_ui() 的成绩页面 -> h_drive_timer_show()
```

以上只是接入设计记录，当前代码尚未执行这些调用。

## 4. 当前 Git 基线

2026-07-30 最新复核时：

```text
branch: master
HEAD:   4160d6b docs: define Raspberry Pi dual controller interfaces
```

近期相关历史：

```text
4160d6b docs: define Raspberry Pi dual controller interfaces
0b555bf chore: remove legacy Tasks4-7 flows
65011bc feat: replace Task3 with offline ball transfer flow
c88185b merge: add standalone H-track drive timer display
c44155b feat: add standalone H-track drive timer display
8a0051b feat: replace Task2 with H lap stop mode
8107c8d chore: remove obsolete ADC implementations
bbcc19c feat: add selectable H-track stop strategies
```

创建后复核时，工作区存在多个其他窗口留下的未提交改动，其中包括：

```text
PROJECT_CONTEXT.md
project/README_RPI_COMM.md
project/README_TASKS_4_7_CLEANUP.md
project/README_TASK1_H_TRACK_PID.md
project/code/app_state.h
project/code/board_comm.c
project/code/board_comm.h
project/code/display.c
project/code/display.h
project/code/tracking_control.c
project/code/tracking_control.h
project/user/src/isr.c
project/user/src/main.c
CODEX_BALL_CONTROL_HANDOFF.md
CODEX_TASK_HANDOFF.md
```

这些状态可能在其他窗口工作时继续变化，以上只用于说明当前工作区并不干净，不应作为固定清单。接手窗口必须以实时 `git status` 为准。不要回退、覆盖、取消暂存或把其他任务文件混入新的功能提交。

本次复核没有发现已暂存文件，但这也可能随其他窗口操作而变化。

## 5. 下一窗口的启动指令

可把下面这段直接发给另一个 Codex 窗口：

```text
请先完整阅读仓库根目录 CODEX_H_TRACK_CONVERSATION.md，再读取
CODEX_TASK_HANDOFF.md 和 PROJECT_CONTEXT.md。随后执行 git status --short --branch，
以当前工作区为准，保留所有既有未提交和已暂存修改。当前先不要修改或接入代码；
先向我汇报三个停车策略、h_drive_timer_* 和 h_task2_* 各自的实际引用状态，
然后严格按照我的下一步指令逐项修改、验证、写提交说明。
```

## 6. 下一步尚未决定

等待用户逐步指定，可能包括：

1. 实车记录 A 点黑线经过时的 8 路 ADC 数据。
2. 决定 Task2 最终采用 355° 直接停、黑线停，还是 Yaw 降速加黑线停。
3. 确定车头或车尾相对 A 点黑线的起始对齐方式。
4. 标定 `slow_yaw_deg`、`slow_speed`、黑线阈值和 `continue_ms`/`StopDly`。
5. 决定保留 `h_task2_*` 计时还是改用 `h_drive_timer_*`。
6. 经用户确认后，才把选定函数接入相应任务并进行 Keil 编译和实车验证。

## 7. Task2 黑线 ADC 实车标定步骤

当前停车判断使用 `adc_calibrated_value[0..7]`，每路范围为 0~100，不直接使用
原始 ADC。默认临时条件为：

```text
单路值 >= 60
有效探头数 >= 4
8 路总和 >= 260
左半区和右半区都至少有一路有效
连续满足 15 ms
```

### 7.1 每次断电后的基础标定

背景值和前景值目前保存在 RAM 中，因此重新上电后应重新标定：

1. 小车停车并进入 OLED 的 ADC/陀螺仪调试页，确认 `Md:0`。
2. 让 8 个探头全部位于赛道白色背景上，短按 KEY1 进入 `Md:1`，保持约
   0.5~1 s。
3. 仍在白色背景上短按 KEY1 进入 `Md:2`，随后把 5 cm 黑色横线移到全部
   8 个探头下方，稳定保持约 0.5~1 s。
4. 保持黑线位置，再短按 KEY1 回到 `Md:0`。
5. 分别放回白底和黑线检查：白底应接近 0，黑线应接近 100；若方向相反、
   多路长期为 0/100 或各路差异很大，应先重新标定，不要开始停车测试。

标定模式会持续刷新当前模式对应的数组，所以在 `Md:1` 时不要长时间把探头
停在黑线上，在 `Md:2` 完成后也应及时切回 `Md:0`。

### 7.2 采集普通线和 A 点横线数据

先把 `StopDly` 设为 0，并保持当前临时阈值不变。低速运行 Task2 时，Task2
页面会实时显示：

```text
Black: 当前达到单路阈值的探头数
Sum:   8 路归一化值总和
Yaw:   当前累计偏航角
```

停车后 `Black` 和 `Sum` 会保留最后一次停车判断的数据。至少做 5 次，记录：

| 次数 | 普通纵向线 Black 最大值 | 普通纵向线 Sum 最大值 | A 点横线 Black | A 点横线 Sum | 是否漏停/误停 |
|---:|---:|---:|---:|---:|---|
| 1 |  |  |  |  |  |
| 2 |  |  |  |  |  |
| 3 |  |  |  |  |  |
| 4 |  |  |  |  |  |
| 5 |  |  |  |  |  |

### 7.3 根据实测值选择阈值

- `h_stop_black_threshold`：先保留 60；只有单路黑白区分明显偏移时再调整。
- `h_stop_black_min_active`：应严格大于普通纵向线的最大有效路数，同时不高于
  5 次 A 点横线中的最小有效路数。
- `h_stop_black_min_sum`：应高于普通纵向线 5 次最大值，并低于 A 点横线
  5 次最小值，优先放在两者中间并保留余量。
- 如果普通线最大值和横线最小值重叠，不要强行选阈值；应先检查探头高度、
  黑白标定、横线覆盖范围和车体经过横线时的姿态。

确定阈值后，再从 `StopDly=0` 开始按 5~10 ms 逐步增加，测量停车基准点
相对 A 点的偏差。阈值解决“在哪里识别”，`StopDly` 解决“识别后车体继续走
多远”，两者应分开标定。
