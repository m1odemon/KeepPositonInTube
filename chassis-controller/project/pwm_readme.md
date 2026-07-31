# PWM 测试说明（与当前工程方向一致）

这份说明用于你新写测试文件时，保证电机方向和当前工程一致，并用于验证“相同 PWM 下左右轮转速是否一致”。

## 1. 当前工程电机引脚映射

来自 `adc_test.h`：

- 右电机方向引脚：`DIR_R = B11`
- 右电机 PWM 引脚：`PWM_R = PWM_TIM_G0_CH0_B10`
- 左电机方向引脚：`DIR_L = A27`
- 左电机 PWM 引脚：`PWM_L = PWM_TIM_G7_CH0_A26`

## 2. 当前工程初始化状态

来自 `Motor_init()`：

- `gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);`
- `pwm_init(PWM_L, 17000, 0);`
- `gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);`
- `pwm_init(PWM_R, 17000, 0);`

说明：

- 上电初始化后，两个 `DIR` 默认是高电平。
- 但真正方向由后续 `motor_set_L/R()` 每次写 `DIR` 决定，初始化高电平不是最终运行方向。

## 3. 当前工程方向电平规则（关键）

来自 `motor_set_R()` 与 `motor_set_L()`：

- 当 `speed >= 0`：`DIR = 0`
- 当 `speed < 0`：`DIR = 1`

即：

- `DIR_L=0`、`DIR_R=0`：当前工程定义的“正转方向”
- `DIR_L=1`、`DIR_R=1`：当前工程定义的“反转方向”

## 4. 做“同 PWM 对比测试”时的建议

为避免受现有补偿逻辑影响（电压补偿、比例缩放、限幅），建议测试代码直接：

1. 固定左右同一 `DIR` 电平（先都设 `0`，再都设 `1` 测一次）。
2. 左右直接下发同一 `pwm_set_duty()` 数值（例如 `2000/3000/4000`）。
3. 分别记录空载与落地两种情况。

注意：当前工程里实际有电压补偿和限幅，`motor_set_L/R()` 内部会改写等效占空比。你若要纯对比，建议测试文件不要复用这段补偿逻辑。

## 5. 最小测试模板（方向与当前工程一致）

```c
void motor_test_init(void)
{
    gpio_init(A27, GPO, GPIO_HIGH, GPO_PUSH_PULL);         // DIR_L
    pwm_init(PWM_TIM_G7_CH0_A26, 17000, 0);                // PWM_L

    gpio_init(B11, GPO, GPIO_HIGH, GPO_PUSH_PULL);         // DIR_R
    pwm_init(PWM_TIM_G0_CH0_B10, 17000, 0);                // PWM_R
}

// dir_level: 0=正转(与当前工程speed>=0一致), 1=反转
void motor_test_set_same(uint8_t dir_level, uint16_t duty)
{
    if (duty > 5000) duty = 5000; // 你当前约定的50%上限

    gpio_set_level(A27, dir_level); // 左
    gpio_set_level(B11, dir_level); // 右

    pwm_set_duty(PWM_TIM_G7_CH0_A26, duty); // 左
    pwm_set_duty(PWM_TIM_G0_CH0_B10, duty); // 右
}
```

## 6. 快速核对清单

- 左电机接线：`A27(GPIODIR)`、`A26(PWM)`、`GND`
- 右电机接线：`B11(GPIODIR)`、`B10(PWM)`、`GND`
- 若“方向相反”，优先检查电机两线是否左右互换或驱动器 A/B 端定义是否反向。
