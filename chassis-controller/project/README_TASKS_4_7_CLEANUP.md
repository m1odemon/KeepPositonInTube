# Task4～Task7 旧工程清理说明

## 当前状态

Task4～Task7 的上次赛题专属入口已经删除，四个菜单位置暂时保留，显示为
`TODO`。在对应 H 题功能逐项实现之前，按 `KEY4` 不会发车。

为了避免状态异常导致误动作，保护分为两层：

1. `display.c` 的任务启动分支拒绝使能 Task4～Task7。
2. `tracking_control.c` 的控制循环及左右电机输出函数强制将 PWM 置零。

## 已删除内容

- Task4～Task7 的旧目标数量校验和目标选择启动入口。
- Task7 等待树莓派放行的 `pi_wait_timer`。
- `cross_state == 1` 的旧 Task7 停车等待分支。
- Task7 从路口状态机进入等待放行状态的跳转。

## 后续清理结果

Task1 已于后续步骤替换为 H题环线循迹，原先暂时保留的 `target_selected[]`、
节点/方向状态、直角转向和 `crossroad_handler()` 现已整体删除。

## 后续接入原则

实现 Task4、Task5、Task6 或 Task7 时，应逐项解除对应任务的禁用保护，并为
新任务建立独立状态机。不要直接恢复 Git 历史中的旧目标选择或树莓派放行流程。
