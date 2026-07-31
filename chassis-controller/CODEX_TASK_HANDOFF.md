# Codex 对话与任务交接

> 更新时间：2026-07-30  
> 仓库：`D:\Codex_TI_NEW\工程模板\SeekFree_common`  
> 当前分支：`master`

## 1. 新窗口启动方式

新 Codex 窗口开始工作前，先读取：

1. 本文件 `CODEX_TASK_HANDOFF.md`；
2. 根目录 `PROJECT_CONTEXT.md`；
3. 当前准备处理任务对应的 `project/README_*.md`；
4. `git status --short --branch`，不要覆盖、回滚或混入已有改动。

可以直接向新窗口发送：

```text
请先完整阅读 CODEX_TASK_HANDOFF.md 和 PROJECT_CONTEXT.md，以当前工作区代码为准。
先检查 git status，保留已有改动和单独暂存的文件。继续完成交接文档中“当前未提交
工作”和“下一步建议”，修改后执行完整 Keil Rebuild，并按任务建立独立 Git 提交。
```

赛题原文：

```text
C:\Users\18371\Downloads\车载平衡滚球运动控制系统（H题）.pdf
```

## 2. 总体目标与约束

- 清理上一次赛题遗留代码，将 Task1～Task7 逐项替换为当前 H 题逻辑。
- 每一步先静态审计，再小范围修改、完整编译、编写说明并独立提交。
- 当前阶段主动断开全部树莓派任务通信；只保留电脑 PID 调参通道。
- Task3、Task4～Task7 在测量链路或新逻辑未完成时必须保持安全，不得猜测传感器
  结果或意外启动底盘。
- 不宣称已经满足时间和误差指标，除非完成实车标定与测量。

## 3. 赛题与程序任务映射

| 程序任务 | 当前定位 |
|---|---|
| Task1 | H题椭圆环线增强循迹，KEY4 启停，不自动停车 |
| Task2 | 从 A 点顺时针一圈并停回 A 点，显示时间 |
| Task3 | 静止小车，钢球 `+5 cm -> -5 cm`；当前无位置源时停在 `WAIT SENSOR` |
| Task4 | A 到 B，钢球保持中心；当前禁用占位 |
| Task5 | 一圈通过 A，钢球保持中心；当前禁用占位 |
| Task6 | 一圈通过 A，钢球保持启动时指定位置；当前禁用占位 |
| Task7 | 赛题“其他”并非规定运动动作；当前禁用占位 |
| Task8 | B4 舵机独立测试，底盘关闭 |
| Task9 | Task1 同类的 H题增强环线测试入口 |
| Task10 | 基础 PID 环轨对照，UART2 电脑在线调参和 CSV 输出 |

## 4. 已完成并提交

| 提交 | 内容 |
|---|---|
| `8107c8d` | 删除未被工程引用的旧 `adc/` 重复实现 |
| `8a0051b` | 将 Task2 替换为 H题一圈停回 A 点 |
| `c44155b` / `c88185b` | 新增并合入独立 H题行驶计时显示 |
| `65011bc` | 将 Task3 替换为离线安全滚球流程，断开树莓派任务通信 |
| `0b555bf` | 删除旧 Task4～Task7 专属流程并设为安全占位 |

当前 `HEAD` 仍为：

```text
0b555bf chore: remove legacy Tasks4-7 flows
```

## 5. 当前已经完成但尚未提交的工作

Task1/Task10 清理已经实现并通过编译，但 Git 写入审批服务连续两次连接中断，
所以尚未创建提交。

已完成内容：

- Task1 改为 H题椭圆环线增强循迹，直接由 KEY4 启停。
- Task10 接管 UART2 ASCII PID 命令和 CSV 状态输出。
- Task1 不再进入目标选择页面，也不解析电脑通信。
- 删除六靶目标选择、丰字节点/方向规划、十字路口前探、90°转向、180°调头、
  转弯停顿和恢复状态。
- 删除 `cross_state`、`target_selected[]`、`current_node`、
  `current_nav_action` 等旧接口。
- `turn_threshold` 已从 OLED 参数列表删除。
- Task2、Task3 和 Task4～Task7 的安全边界保持不变。
- `main.c`、`isr.c` 原本是混合损坏编码；为应用补丁已机械规范为 UTF-8，
  因此差异中包含部分旧注释字符替换，运行逻辑变化仅是 Task10 通信判断。

本次新增说明：

```text
project/README_TASK1_H_TRACK_PID.md
```

最新完整 Keil Rebuild：

```text
Program Size: Code=60532 RO-data=4768 RW-data=412 ZI-data=6748
0 Error(s), 0 Warning(s)
```

构建命令：

```powershell
& 'D:\Keil5\UV4\UV4.exe' -r `
  '.\project\keil\SeekFree_MSPM0G3507_Device_Library.uvprojx' -j0
```

## 6. 重要 Git 状态

当前存在一份不属于本次 Task1 修改的已暂存文件：

```text
M  project/README_BALL_RPI_TEAM.md
```

该文件包含另一组较大的树莓派协作文档修改。不要取消暂存、覆盖或混入 Task1
提交。本次提交必须使用 `git commit --only` 和明确文件列表。

Task1 提交文件列表应为：

```powershell
$task1Files = @(
  'CODEX_TASK_HANDOFF.md',
  'PROJECT_CONTEXT.md',
  'project/README_RPI_COMM.md',
  'project/README_TASKS_4_7_CLEANUP.md',
  'project/README_TASK1_H_TRACK_PID.md',
  'project/code/app_state.h',
  'project/code/board_comm.c',
  'project/code/board_comm.h',
  'project/code/display.c',
  'project/code/display.h',
  'project/code/tracking_control.c',
  'project/code/tracking_control.h',
  'project/user/src/main.c',
  'project/user/src/isr.c'
)
```

推荐提交：

```powershell
git add -- $task1Files
git diff --cached --check -- $task1Files
git commit --only `
  -m 'refactor: replace Task1 with H-track mode' `
  -m 'Task1改为H题环线增强循迹；PID电脑通信迁移到Task10；删除旧丰字目标与转向状态机，并同步操作说明。' `
  -- $task1Files
```

提交后应确认：

```powershell
git show --stat --oneline --summary HEAD
git status --short --branch
```

预期 Task1 文件全部干净，但 `project/README_BALL_RPI_TEAM.md` 仍保持单独暂存。

## 7. 当前关键运行行为

### Task1

- OLED：`1:H Track`
- 使用 `curve_tracking_control()`、趋势前瞻、曲率保持、动态降速和自适应差速。
- 与 Task9 使用相同底盘算法。
- 不自动计圈或停车；KEY4 或电池安全保护停止。

### Task10

- OLED：`10:PID UART`
- 使用基础 `standard_tracking_shape_line_error()` 作为对照。
- 仅选中 Task10 时，UART2 中断解析电脑 ASCII 命令：

```text
STATUS
SET P:7.0 I:0 D:0.2
```

- 主循环发送 CSV：

```text
timestamp,setpoint,input,pwm,error,Kp,Ki,Kd
```

- `PIDK_YA` 为全局共享参数。Task10 调整后切换 Task1/Task2/Task9，数值仍然
  生效；但增强误差整形不同，必须重新低速验证。

### Task2

- 使用 H题增强环线循迹。
- 累计偏航约 300° 后降速并布防 A 点宽线。
- 当前宽线阈值和 `StopDly` 仍需实车标定。

### Task3 与 Task4～Task7

- Task3 没有钢球位置源时显示 `WAIT SENSOR`，底盘和舵机保持安全。
- Task4～Task7 的菜单位置存在，但 KEY4 不发车。
- 控制循环和左右电机输出函数均对 Task4～Task7 设置 PWM 硬锁。

## 8. 下一步建议

1. 优先完成上述 Task1 独立 Git 提交。
2. 架空车轮验证 Task1、Task10 的 KEY4 急停和电机方向。
3. 在 Task10 连接电脑，验证 `STATUS`、`SET`、ACK 和 CSV。
4. 低速分别复测 Task10 基础 PID 与 Task1 增强环线。
5. 然后建立 Task4～Task6 共用的“环线运动 + 钢球控制”框架：
   - Task4：B 点结束、中心目标、8 s；
   - Task5：A 点一圈结束、中心目标、30 s；
   - Task6：A 点一圈结束、启动位置锁存目标、30 s。
6. 树莓派通信最后统一接入钢球测量适配口，避免三个任务分别写协议。

## 9. 本对话的环线实测记录与 PID 决策

### 9.1 赛道与车辆

- 赛题轨道：两段约 `1.5 m` 直线连接两个半径约 `0.5 m` 的半圆。
- 黑线宽度：`1.8 ± 0.2 cm`。
- 用户车辆尺寸：约 `30 cm × 20 cm`。
- A 点带一条与环线垂直的横向启停线；持续环轨任务必须将它作为普通线路通过。

### 9.2 已完成的实车对照

在相同右弯上已经得到以下结果：

| 测试模式 | 控制算法 | 实车结果 |
|---|---|---|
| Task9 旧增强版 | 趋势前瞻、动态降速、自适应差速 | 约行驶 `1/4` 圆弧后从外侧冲出 |
| Task10 基础版 | Task1～7 原直线 PID 误差整形 | 约行驶 `1/8` 圆弧后从外侧冲出 |

结论：增强算法方向有效，但当时仍缺少维持恒定圆弧所需的持续右转角速度；不能仅
通过扩大“允许差速上限”解决，因为 PID 实际命令可能没有长期达到该上限。

提交 `9a31a67 feat: add PID oval comparison and curve hold` 随后加入：

- Task9 有界曲率保持量；
- `CURVE_CONTROL_GAIN = 1.65`；
- 最大动态差速 `9.0`；
- 弯中最低速度系数 `0.72`；
- OLED 调试量 `Cmd`（目标角速度）和 `Dif`（实际差速）。

第三版随后完成实车复测，用户确认 **Task9 已经可以正常循迹通过弯道**。这说明
当前曲率保持方向、Task9 专用等效外环增益和动态差速组合已经达到可用基线。
当前工作区重构后，Task1、Task2、Task9 使用同一增强算法，Task10 继续保留基础
PID 对照。

目前尚未确认的项目包括：连续多圈稳定性、完整直线—弯道—直线出弯收敛、外力
扰动恢复、不同电池电压和左右方向一致性。在完成这些项目之前，不宣称整套赛题
循迹已经最终标定完成。

### 9.3 当前 PID 与下一轮判断

当前全局参数：

```text
外环 PIDK_YA = { Kp=7.0, Ki=0, Kd=0.2 }
内环 PIDK_YG = { Kp=0.5, Ki=0, Kd=0 }
```

当前 Task9 已能正常通过弯道，不要继续无依据地放大全局 PID。Task10 在线修改的
是全局 `PIDK_YA`，切换到 Task1、Task2、Task9 后仍然生效，可能破坏已经可用的
直线和弯道行为。

下一轮优先低速测试 Task1 或 Task9，并记录脱轨前：

```text
Err / Cmd / Yg / Dif / 菜单 Speed
```

右弯按当前符号链通常应为：

```text
Err < 0
Cmd > 0
Dif < 0
```

判断规则：

- `Dif` 已接近 `-9.0` 仍向外冲：控制输出已饱和，优先降低实际速度，或再扩大
  物理差速/降低内侧轮下限，不要先加外环 PID。
- `Cmd` 偏小且 `Dif` 远未到 `-9.0`：只增强 H题任务的外环等效增益或曲率保持量。
- `Cmd` 较大、`Yg` 明显跟不上且 `Dif` 未饱和：再考虑只为 H题任务提高角速度
  内环增益，避免改变 Task10 对照基线。
- 开始蛇形或切入弯道内侧：曲率保持或外环增益过强，应回调而不是继续放大。

新窗口若要继续本对话的环线任务，可直接发送：

```text
请阅读 CODEX_TASK_HANDOFF.md 的第 9 节，并以当前工作区为准继续 H题环线实车调试。
先检查 git status，不覆盖当前未提交的 Task1/Task10 重构，也不要混入已单独暂存的
README_BALL_RPI_TEAM.md。Task9 第三版已经能正常通过弯道，先冻结当前 PID 参数并
验证完整出弯、连续多圈和抗扰性能；只有出现可重复问题时才结合 Err/Cmd/Yg/Dif 调参。
```

## 10. 禁止误操作

- 不要执行 `git reset --hard`、`git checkout --` 或清理整个工作区。
- 不要把已暂存的 `README_BALL_RPI_TEAM.md` 混入 Task1 提交。
- 不要重新引入 Git 历史中的丰字赛道或旧树莓派放行协议。
- 不要在没有有效、及时钢球位置数据时启动 Task4～Task6。
- 编译通过只代表软件完整，不代表已经满足赛题时间与误差指标。
