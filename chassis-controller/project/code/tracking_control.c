#include "tracking_control.h"
#include "zf_common_headfile.h"
#include "display.h"
#include "app_state.h"  // 获取 run_state, motor_enable, battery_voltage
#include "battery_3s.h"
#include "icm.h"     // 获取 Yaw_TotalAngle, Yaw_g

// =========================================================================
// 第一部分：变量定义
// =========================================================================
volatile PID_t PIDK_YG = { 0.5, 0, 0 },
PIDK_YA = { 7.0f, 0, 0.2f };

float aim_yaw_g;
float Duty_dS;
float Duty_dS_applied = 0.0f;
float fclip(float x, float low, float up) { return x > up ? up : x < low ? low : x; }

/*
 * Unified straight/curve tracking parameters for the 5 ms control loop.
 *
 * The H-track has 0.5 m radius semicircles.  A roughly 0.20 m wide chassis
 * needs about 20% left/right wheel-speed difference to follow that radius.
 * The controller therefore slows down progressively while allowing a larger
 * differential command, instead of waiting for a large line error and then
 * applying an abrupt correction.
 */
#define CURVE_SIGNAL_ENTER_SUM       (45.0f)
#define CURVE_SIGNAL_EXIT_SUM        (25.0f)
#define CURVE_SIGNAL_CONFIRM_TICKS   (3U)
#define CURVE_RATE_FILTER_ALPHA      (0.25f)
#define CURVE_RATE_DEADZONE          (0.008f)
#define CURVE_RATE_LIMIT             (0.28f)
#define CURVE_LOOKAHEAD_GAIN         (2.80f)
#define CURVE_LOOKAHEAD_LIMIT        (0.90f)
#define CURVE_OPPOSING_COMP_RATIO    (0.85f)
#define CURVE_ERROR_LIMIT            (3.50f)
#define CURVE_ERROR_DEMAND_START     (0.08f)
#define CURVE_ERROR_DEMAND_FULL      (1.08f)
#define CURVE_TREND_DEMAND_START     (0.006f)
#define CURVE_TREND_DEMAND_FULL      (0.096f)
#define CURVE_SEVERITY_ATTACK        (0.20f)
#define CURVE_SEVERITY_RELEASE       (0.012f)
#define CURVE_SPEED_FACTOR_MIN       (0.72f)
#define CURVE_LOST_SPEED_FACTOR      (0.70f)
#define CURVE_DUTY_LIMIT_BASE        (5.20f)
#define CURVE_DUTY_LIMIT_MAX         (9.00f)
#define TRACKING_MIN_WHEEL_SPEED     (24.0f)
#define CURVE_MIN_WHEEL_SPEED        (16.0f)
#define CURVE_HOLD_ERROR_MIN         (0.08f)
#define CURVE_HOLD_ACCUM_GAIN        (0.010f)
#define CURVE_HOLD_LIMIT             (1.60f)
#define CURVE_HOLD_LEAK              (0.999f)
#define CURVE_HOLD_OPPOSING_DECAY    (0.82f)
#define CURVE_CONTROL_GAIN           (1.65f)

#define TRACKING_CENTER_DEADZONE     (0.05f)
#define TRACKING_CENTER_MIN_GAIN     (0.68f)
#define TRACKING_FULL_GAIN_ERROR     (0.28f)
#define TRACKING_HIGH_GAIN_START     (1.40f)
#define TRACKING_HIGH_GAIN_MAX       (1.15f)

static float curve_last_error = 0.0f;
static float curve_error_rate = 0.0f;
static float curve_hold_bias = 0.0f;
static float curve_tracking_severity = 0.0f;
static float curve_tracking_speed_factor = 1.0f;
static uint8_t curve_history_valid = 0;
static uint8_t curve_signal_valid = 0;
static uint8_t curve_signal_transition_ticks = 0;

static void curve_tracking_clear_history(void)
{
    curve_last_error = 0.0f;
    curve_error_rate = 0.0f;
    curve_history_valid = 0;
}

static void curve_tracking_reset(void)
{
    curve_tracking_clear_history();
    curve_hold_bias = 0.0f;
    curve_tracking_severity = 0.0f;
    curve_tracking_speed_factor = 1.0f;
    curve_signal_valid = 0;
    curve_signal_transition_ticks = 0;
}

float speed_set_duty = 40.0f;
float speed_set = 0.0f;
float speed_set_max = 80.0f;
float speed_set_diff_L = 0.0f;
float speed_set_diff_R = 0.0f;
float speed_set_diff_R_last = 0.0f;
float speed_set_diff_L_last = 0.0f;
int start_delay = 0;

// 临时测试开关：1=开环同PWM直行测试，0=恢复正常PID循迹
#define PWM_EQUAL_TEST_ENABLE (0)
#define PWM_EQUAL_TEST_DUTY   (2000)

// =========================================================================
// 直线/曲线统一循迹前瞻
// =========================================================================
// 单排灰度无法在单帧内区分“直线横向偏移”和“进入弯道”，也不需要硬分类。
// 这里连续观察误差趋势：误差继续向同一方向增长时提前加转向，误差回落时
// 提前收小转向。返回值仍是等效循迹误差，不改变已有串级 PID 的方向。
static uint8_t curve_tracking_update_signal_valid(float line_strength)
{
    if (curve_signal_valid) {
        if (line_strength <= CURVE_SIGNAL_EXIT_SUM) {
            if (++curve_signal_transition_ticks >= CURVE_SIGNAL_CONFIRM_TICKS) {
                curve_signal_valid = 0;
                curve_signal_transition_ticks = 0;
            }
        } else {
            curve_signal_transition_ticks = 0;
        }
    } else {
        if (line_strength >= CURVE_SIGNAL_ENTER_SUM) {
            if (++curve_signal_transition_ticks >= CURVE_SIGNAL_CONFIRM_TICKS) {
                curve_signal_valid = 1;
                curve_signal_transition_ticks = 0;
            }
        } else {
            curve_signal_transition_ticks = 0;
        }
    }

    return curve_signal_valid;
}

float curve_tracking_control(void)
{
    float line_strength = 0.0f;
    for (int i = 0; i < 8; i++) {
        line_strength += (float)adc_calibrated_value[i];
    }

    // 使用带迟滞和连续计数的有效线判断，避免在阈值附近反复启停前瞻。
    if (!curve_tracking_update_signal_valid(line_strength)) {
        curve_tracking_clear_history();
        // 已经滑到黑线边缘时不能同时收掉差速权限；保留最大纠偏能力，
        // 配合降速沿最后一个有效误差方向找回线路。
        curve_tracking_severity = 1.0f;
        curve_tracking_speed_factor = CURVE_LOST_SPEED_FACTOR;
        return fclip(line_error_filtered, -CURVE_ERROR_LIMIT, CURVE_ERROR_LIMIT);
    }

    // 首个有效采样只建立历史，不注入由上一个状态产生的伪前瞻量。
    if (!curve_history_valid) {
        curve_last_error = line_error_filtered;
        curve_error_rate = 0.0f;
        curve_history_valid = 1;
        curve_tracking_severity = 0.0f;
        curve_tracking_speed_factor = 1.0f;
        return fclip(line_error_filtered, -CURVE_ERROR_LIMIT, CURVE_ERROR_LIMIT);
    }

    float error_delta = line_error_filtered - curve_last_error;
    if (fabsf(error_delta) <= CURVE_RATE_DEADZONE) {
        error_delta = 0.0f;
    } else {
        error_delta -= (error_delta > 0.0f)
                     ? CURVE_RATE_DEADZONE
                     : -CURVE_RATE_DEADZONE;
    }
    error_delta = fclip(error_delta, -CURVE_RATE_LIMIT, CURVE_RATE_LIMIT);
    curve_error_rate += CURVE_RATE_FILTER_ALPHA * (error_delta - curve_error_rate);
    curve_last_error = line_error_filtered;

    // 单排传感器进入恒定圆弧后，误差变化率会逐渐回到零；仅靠前瞻会丢失
    // 维持圆弧所需的角速度。这里对同方向持续误差做有界积分，形成曲率保持量。
    // 出弯或受外力回正时，一旦误差反向便快速释放，避免把弯道记忆带入直线。
    if (fabsf(line_error_filtered) > CURVE_HOLD_ERROR_MIN) {
        if (curve_hold_bias * line_error_filtered < 0.0f) {
            curve_hold_bias *= CURVE_HOLD_OPPOSING_DECAY;
        } else {
            curve_hold_bias += CURVE_HOLD_ACCUM_GAIN * line_error_filtered;
        }
    } else {
        curve_hold_bias *= CURVE_HOLD_LEAK;
    }
    curve_hold_bias = fclip(curve_hold_bias, -CURVE_HOLD_LIMIT, CURVE_HOLD_LIMIT);
    if (fabsf(curve_hold_bias) < 0.002f) {
        curve_hold_bias = 0.0f;
    }

    float error_demand = fclip((fabsf(line_error_filtered) - CURVE_ERROR_DEMAND_START)
                               / (CURVE_ERROR_DEMAND_FULL - CURVE_ERROR_DEMAND_START),
                               0.0f,
                               1.0f);
    float trend_demand = fclip((fabsf(curve_error_rate) - CURVE_TREND_DEMAND_START)
                               / (CURVE_TREND_DEMAND_FULL - CURVE_TREND_DEMAND_START),
                               0.0f,
                               1.0f);
    float severity_target = (trend_demand > error_demand * 0.65f)
                          ? trend_demand
                          : error_demand * 0.65f;
    float hold_demand = fabsf(curve_hold_bias) / CURVE_HOLD_LIMIT;
    if (hold_demand > severity_target) {
        severity_target = hold_demand;
    }
    float severity_alpha = (severity_target > curve_tracking_severity)
                         ? CURVE_SEVERITY_ATTACK
                         : CURVE_SEVERITY_RELEASE;
    curve_tracking_severity += severity_alpha
                             * (severity_target - curve_tracking_severity);
    curve_tracking_severity = fclip(curve_tracking_severity, 0.0f, 1.0f);
    curve_tracking_speed_factor = 1.0f
                                - (1.0f - CURVE_SPEED_FACTOR_MIN)
                                * curve_tracking_severity;

    // 偏差 + 有限前瞻：入弯提前加转向，出弯提前收力，直线稳定后前瞻自然归零。
    float lookahead = fclip(CURVE_LOOKAHEAD_GAIN * curve_error_rate,
                            -CURVE_LOOKAHEAD_LIMIT,
                            CURVE_LOOKAHEAD_LIMIT);
    if (line_error_filtered == 0.0f) {
        lookahead = 0.0f;
    } else if (line_error_filtered * lookahead < 0.0f) {
        float opposing_limit = fabsf(line_error_filtered)
                             * CURVE_OPPOSING_COMP_RATIO;
        lookahead = fclip(lookahead, -opposing_limit, opposing_limit);
    }
    float curve_error = line_error_filtered + lookahead + curve_hold_bias;
    return fclip(curve_error, -CURVE_ERROR_LIMIT, CURVE_ERROR_LIMIT);
}

static float tracking_shape_line_error(float error)
{
    float magnitude = fabsf(error);
    float shaped_magnitude;

    if (magnitude <= TRACKING_CENTER_DEADZONE) {
        return 0.0f;
    }

    if (magnitude < TRACKING_FULL_GAIN_ERROR) {
        float transition = (magnitude - TRACKING_CENTER_DEADZONE)
                         / (TRACKING_FULL_GAIN_ERROR - TRACKING_CENTER_DEADZONE);
        float center_gain;
        // 小偏差仍保留基础纠偏力，避免受外力后“看见偏差却回得太慢”。
        transition = transition * transition * (3.0f - 2.0f * transition);
        center_gain = TRACKING_CENTER_MIN_GAIN
                    + (1.0f - TRACKING_CENTER_MIN_GAIN) * transition;
        shaped_magnitude = magnitude * center_gain;
    } else {
        shaped_magnitude = magnitude;
    }

    if (magnitude > TRACKING_HIGH_GAIN_START) {
        float high_error_ratio = (magnitude - TRACKING_HIGH_GAIN_START)
                               / (CURVE_ERROR_LIMIT - TRACKING_HIGH_GAIN_START);
        float high_error_gain = 1.0f
                              + (TRACKING_HIGH_GAIN_MAX - 1.0f)
                              * fclip(high_error_ratio, 0.0f, 1.0f);
        shaped_magnitude *= high_error_gain;
    }

    return (error >= 0.0f) ? shaped_magnitude : -shaped_magnitude;
}

// Task10 保留 H 题增强算法接入前的误差整形，便于电脑在线调参对照。
static float standard_tracking_shape_line_error(float error)
{
    float shaped_error = error;
    float magnitude = fabsf(shaped_error);

    if (magnitude < 0.10f) {
        shaped_error = 0.0f;
    } else if (magnitude < 0.35f) {
        shaped_error *= 0.65f;
    } else if (magnitude > 1.40f) {
        shaped_error *= 1.15f;
    }

    return shaped_error;
}


// =========================================================================
// 第四部分：控制器与发车重置
// =========================================================================
float YAError[2] = {0.0f}, YAIntegral[2] = {0.0f};
float YAError_last[2] = {0.0f}, aim_yaw_g_last[2] = {0.0f};
float YGIntegral = 0.0f;
float dYAError_MAX = 1.0f;
//相比原函数多了一些部分
void reset_pid(void)
{
    curve_tracking_reset();
    YGIntegral = 0.0f;
    Duty_dS = 0.0f;
    Duty_dS_applied = 0.0f;
    for(int i = 0; i < 2; i++) {
        YAError[i] = 0.0f;
        YAIntegral[i] = 0.0f;
        YAError_last[i] = 0.0f;
        aim_yaw_g_last[i] = 0.0f;
    }
}

//小车初始化，每次发车前调用
void car_start_init(void)
{
    battery_3s_reset_if_recovered(battery_voltage);

    speed_set_diff_L = 40.0f;
    speed_set_diff_R = 40.0f;
    speed_set_duty = 40.0f;
    speed_set_diff_L_last = 0.0f;
    speed_set_diff_R_last = 0.0f;

    reset_pid();
}

// =========================================================================
// H题停车策略与Task2一圈停车状态
// =========================================================================
#define H_STOP_CONTROL_TICK_MS          (5U)
#define H_STOP_BLACK_CLEAR_TICKS        (10U)   // 连续50 ms未见宽线后允许检测
#define H_STOP_BLACK_CONFIRM_TICKS      (3U)    // 连续15 ms见宽线才确认
#define H_STOP_BLACK_MIN_ARM_TICKS      (200U)  // 发车至少1 s后才允许布防
#define H_TASK2_SLOW_YAW_DEG             (300.0f)
#define H_TASK2_SLOW_SPEED               (20.0f)
#define H_STOP_CH3_ADC_INDEX              (2U)
#define H_STOP_CH4_ADC_INDEX              (3U)
#define H_STOP_CH5_ADC_INDEX              (4U)

/*
 * 实车确认A点横向黑线由CH3、CH4、CH5识别，因此只使用这三路
 * 归一化ADC之和判断。三路最大和为300；240是尚待重复实测的保守初值，
 * 用来避免普通纵向线只覆盖中间一至两路时误触发。
 */
int h_stop_ch345_sum_threshold = 240;
volatile int h_stop_ch345_sum = 0;
volatile uint32_t h_task2_elapsed_ms = 0U;
volatile uint8_t h_task2_run_finished = 0U;
volatile int h_task2_stop_delay_ms = 0;

enum
{
    H_STOP_BLACK_WAIT_CLEAR = 0,
    H_STOP_BLACK_ARMED,
    H_STOP_BLACK_DELAY,
    H_STOP_BLACK_FINISHED
};

static uint8_t h_stop_initialized = 0U;
static uint8_t h_stop_finished = 0U;
static uint8_t h_stop_black_state = H_STOP_BLACK_WAIT_CLEAR;
static uint16_t h_stop_black_clear_ticks = 0U;
static uint16_t h_stop_black_confirm_ticks = 0U;
static uint32_t h_stop_elapsed_ticks = 0U;
static uint32_t h_stop_delay_ticks_remaining = 0U;
static float h_stop_start_yaw = 0.0f;

static void h_stop_ensure_initialized(void)
{
    if (!h_stop_initialized) {
        h_stop_strategy_reset();
    }
}

static uint8_t h_stop_black_line_present(void)
{
    int ch345_sum = adc_calibrated_value[H_STOP_CH3_ADC_INDEX]
                  + adc_calibrated_value[H_STOP_CH4_ADC_INDEX]
                  + adc_calibrated_value[H_STOP_CH5_ADC_INDEX];

    h_stop_ch345_sum = ch345_sum;
    return (uint8_t)(ch345_sum >= h_stop_ch345_sum_threshold);
}

static uint8_t h_stop_apply(void)
{
    if (!h_stop_finished) {
        h_stop_finished = 1U;
        h_stop_black_state = H_STOP_BLACK_FINISHED;
        Duty_dS = 0.0f;
        Duty_dS_applied = 0.0f;
        speed_set_diff_R = 0.0f;
        speed_set_diff_L = 0.0f;
        reset_pid();

        motor_enable = 0;
        run_state = 0;
        motor_set_R(0.0f);
        motor_set_L(0.0f);
        beep_set_time(500);
    }

    return 1U;
}

void h_stop_strategy_reset(void)
{
    h_stop_start_yaw = Yaw_TotalAngle;
    h_stop_initialized = 1U;
    h_stop_finished = 0U;
    h_stop_black_state = H_STOP_BLACK_WAIT_CLEAR;
    h_stop_black_clear_ticks = 0U;
    h_stop_black_confirm_ticks = 0U;
    h_stop_elapsed_ticks = 0U;
    h_stop_delay_ticks_remaining = 0U;
    h_stop_ch345_sum = 0;
}

void h_task2_prepare_start(void)
{
    h_task2_elapsed_ms = 0U;
    h_task2_run_finished = 0U;
    h_stop_strategy_reset();
}

/*
 * 方案一：从复位时刻起，累计Yaw绝对变化达到355°后直接停车。
 * 该策略只判断车头累计转角，不保证车辆位置正好回到A点。
 */
uint8_t h_stop_by_yaw_355(void)
{
    h_stop_ensure_initialized();
    if (h_stop_finished) return 1U;

    if (fabsf(Yaw_TotalAngle - h_stop_start_yaw) >= 355.0f) {
        return h_stop_apply();
    }
    return 0U;
}

/*
 * 方案二：检测到A点5 cm横向黑线后停车。
 * continue_ms=0时立即停车；continue_ms=2000时继续循迹2 s后停车。
 */
static uint8_t h_stop_by_black_line_gated(uint32_t continue_ms,
                                          uint8_t detection_enabled)
{
    uint8_t line_present;

    h_stop_ensure_initialized();
    if (h_stop_finished) return 1U;

    if (h_stop_elapsed_ticks < UINT32_MAX) {
        h_stop_elapsed_ticks++;
    }
    line_present = h_stop_black_line_present();

    switch (h_stop_black_state) {
    case H_STOP_BLACK_WAIT_CLEAR:
        if (line_present) {
            h_stop_black_clear_ticks = 0U;
        } else if (h_stop_black_clear_ticks < H_STOP_BLACK_CLEAR_TICKS) {
            h_stop_black_clear_ticks++;
        }

        if (h_stop_elapsed_ticks >= H_STOP_BLACK_MIN_ARM_TICKS &&
            h_stop_black_clear_ticks >= H_STOP_BLACK_CLEAR_TICKS) {
            h_stop_black_state = H_STOP_BLACK_ARMED;
            h_stop_black_confirm_ticks = 0U;
        }
        break;

    case H_STOP_BLACK_ARMED:
        if (!detection_enabled) {
            h_stop_black_confirm_ticks = 0U;
        } else if (line_present) {
            if (h_stop_black_confirm_ticks < H_STOP_BLACK_CONFIRM_TICKS) {
                h_stop_black_confirm_ticks++;
            }
        } else {
            h_stop_black_confirm_ticks = 0U;
        }

        if (h_stop_black_confirm_ticks >= H_STOP_BLACK_CONFIRM_TICKS) {
            if (continue_ms > 60000U) continue_ms = 60000U;
            h_stop_delay_ticks_remaining =
                continue_ms / H_STOP_CONTROL_TICK_MS;
            if ((continue_ms % H_STOP_CONTROL_TICK_MS) != 0U) {
                h_stop_delay_ticks_remaining++;
            }

            if (h_stop_delay_ticks_remaining == 0U) {
                return h_stop_apply();
            }
            h_stop_black_state = H_STOP_BLACK_DELAY;
        }
        break;

    case H_STOP_BLACK_DELAY:
        if (h_stop_delay_ticks_remaining > 0U) {
            h_stop_delay_ticks_remaining--;
        }
        if (h_stop_delay_ticks_remaining == 0U) {
            return h_stop_apply();
        }
        break;

    case H_STOP_BLACK_FINISHED:
    default:
        return h_stop_apply();
    }

    return 0U;
}

uint8_t h_stop_by_black_line(uint32_t continue_ms)
{
    return h_stop_by_black_line_gated(continue_ms, 1U);
}

/*
 * 方案三：累计Yaw达到slow_yaw_deg后把基础速度降到slow_speed，
 * 随后检测A点横线；检测后的停车延时由continue_ms决定。
 * 示例：h_stop_by_yaw_slow_black_line(300.0f, 20.0f, 0U)
 *       h_stop_by_yaw_slow_black_line(330.0f, 20.0f, 2000U)
 */
uint8_t h_stop_by_yaw_slow_black_line(float slow_yaw_deg,
                                      float slow_speed,
                                      uint32_t continue_ms)
{
    float yaw_delta;

    h_stop_ensure_initialized();
    if (h_stop_finished) return 1U;

    yaw_delta = fabsf(Yaw_TotalAngle - h_stop_start_yaw);
    if (yaw_delta < fabsf(slow_yaw_deg)) {
        return h_stop_by_black_line_gated(continue_ms, 0U);
    }

    speed_set_duty = fclip(slow_speed, 0.0f, speed_set_max);
    return h_stop_by_black_line_gated(continue_ms, 1U);
}

static uint8_t h_task2_update_5ms(void)
{
    int stop_delay_ms = h_task2_stop_delay_ms;

    if (h_task2_elapsed_ms <= UINT32_MAX - H_STOP_CONTROL_TICK_MS) {
        h_task2_elapsed_ms += H_STOP_CONTROL_TICK_MS;
    }

    if (stop_delay_ms < 0) stop_delay_ms = 0;
    if (stop_delay_ms > 500) stop_delay_ms = 500;

    if (h_stop_by_yaw_slow_black_line(H_TASK2_SLOW_YAW_DEG,
                                      H_TASK2_SLOW_SPEED,
                                      (uint32_t)stop_delay_ms)) {
        h_task2_run_finished = 1U;
        switch_page = 1;
        return 1U;
    }

    return 0U;
}

// =========================================================================
// 第五部分：底层控制与电机驱动 (完全复刻原作者)
// =========================================================================
float PID_Yaw_a(uint8_t cnl, float YAError_input)
{
    float dYAError, d_aim_yaw_g, output_yaw_g; // 全部变成普通的局部运算变量
    float aim_yaw_g_MAX = 600.0f;

    YAError[cnl] = fclip(YAError_input, -13.0f, 13.0f);
    dYAError = YAError[cnl] - YAError_last[cnl];

    // 微分限幅
    if (fabsf(dYAError) > dYAError_MAX) {
        dYAError = (dYAError > 0) ? dYAError_MAX : -dYAError_MAX;
        YAError[cnl] = YAError_last[cnl] + dYAError;
    }

    YAIntegral[cnl] = fclip(YAIntegral[cnl] + PIDK_YA.Ki * YAError[cnl], -1200.0f, 1200.0f);
    output_yaw_g = fclip(PIDK_YA.Kp * YAError[cnl] + YAIntegral[cnl] + PIDK_YA.Kd * dYAError, -aim_yaw_g_MAX, aim_yaw_g_MAX);

    // 输出限幅
    d_aim_yaw_g = output_yaw_g - aim_yaw_g_last[cnl];
    if (fabsf(d_aim_yaw_g) > 5.0f) {
        output_yaw_g = aim_yaw_g_last[cnl]
                     + ((d_aim_yaw_g > 0) ? 4.0f : -4.0f);
    }

    // 刷新全局记忆
    aim_yaw_g_last[cnl] = output_yaw_g;
    YAError_last[cnl] = YAError[cnl];

    return output_yaw_g;
}

void PID_Yaw_gyro(void) {
    static float YGError, YGError_last;
    YGError = aim_yaw_g - Yaw_g;
    YGIntegral = fclip(YGIntegral + YGError, -7000.0f, 7000.0f);
    Duty_dS = -(PIDK_YG.Kp * YGError + PIDK_YG.Ki * YGIntegral + PIDK_YG.Kd * (YGError - YGError_last));
    YGError_last = YGError;
}

void motor_set_R(float speed) {
    // H题Task3和尚未重写的Task4~7禁止底盘运动，作为UI之外的底层硬锁。
    if (task_number == TASK_H_STATIC_BALL_TRANSFER ||
        (task_number >= TASK_H_DISABLED_FIRST &&
         task_number <= TASK_H_DISABLED_LAST)) {
        pwm_set_duty(PWM_R, 0);
        return;
    }
    // ⭐️ 上帝防线：只要总闸被拉下（比如紧急停车、发车前、比赛结束）
    // 无视任何速度计算，强制切断电机物理电源！
    if (motor_enable == 0) {
        pwm_set_duty(PWM_R, 0);
        return;
    }
    float abs_speed = fabsf(speed);
    abs_speed *= speed_scale;
    uint32 base_duty = (uint32)(abs_speed * 30.0f);
    if (base_duty > MOTOR_3S_BASE_DUTY_LIMIT) {
        base_duty = MOTOR_3S_BASE_DUTY_LIMIT;
    }
    uint32 duty = battery_3s_compensate_motor_duty(base_duty, battery_voltage);

    if (speed >= 0.0f) {
        gpio_set_level(DIR_R, 0);
        pwm_set_duty(PWM_R, duty);
    } else {
        gpio_set_level(DIR_R, 1);
        pwm_set_duty(PWM_R, duty);
    }
}

void motor_set_L(float speed) {
    // H题Task3和尚未重写的Task4~7禁止底盘运动，作为UI之外的底层硬锁。
    if (task_number == TASK_H_STATIC_BALL_TRANSFER ||
        (task_number >= TASK_H_DISABLED_FIRST &&
         task_number <= TASK_H_DISABLED_LAST)) {
        pwm_set_duty(PWM_L, 0);
        return;
    }
    // ⭐️ 上帝防线：只要总闸被拉下（比如紧急停车、发车前、比赛结束）
    // 无视任何速度计算，强制切断电机物理电源！
    if (motor_enable == 0) {
        pwm_set_duty(PWM_L, 0);
        return;
    }
    float abs_speed = fabsf(speed);
    abs_speed *= speed_scale;
    uint32 base_duty = (uint32)(abs_speed * 30.0f);
    if (base_duty > MOTOR_3S_BASE_DUTY_LIMIT) {
        base_duty = MOTOR_3S_BASE_DUTY_LIMIT;
    }
    uint32 duty = battery_3s_compensate_motor_duty(base_duty, battery_voltage);

    if (speed >= 0.0f) {
        gpio_set_level(DIR_L, 0);
        pwm_set_duty(PWM_L, duty);
    } else {
        gpio_set_level(DIR_L, 1);
        pwm_set_duty(PWM_L, duty);
    }


}

void Motor_init(void) {
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_L, 17000, 0);
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_R, 17000, 0);
}

static uint8_t task_uses_enhanced_h_tracking(void)
{
    return (uint8_t)(task_number == TASK_H_BASE_TRACK ||
                     task_number == TASK_H_ONE_LAP_STOP ||
                     task_number == TASK_H_OVAL_TRACK);
}

void motor_control(void)
{
    uint8_t h_oval_tracking = task_uses_enhanced_h_tracking();
    float speed_change_rate_max = 1.0f;
    float min_wheel_speed = TRACKING_MIN_WHEEL_SPEED;
    float duty_limit = 4.8f;

    speed_set = speed_set_duty;
    if (h_oval_tracking) {
        speed_set *= curve_tracking_speed_factor;
        min_wheel_speed -= (TRACKING_MIN_WHEEL_SPEED - CURVE_MIN_WHEEL_SPEED)
                         * curve_tracking_severity;

        {
            float adaptive_duty_limit = CURVE_DUTY_LIMIT_BASE
                                      + (CURVE_DUTY_LIMIT_MAX - CURVE_DUTY_LIMIT_BASE)
                                      * curve_tracking_severity;
            if (adaptive_duty_limit > duty_limit) {
                duty_limit = adaptive_duty_limit;
            }
        }
    }

    // 无直角路口和原地转向后，环线循迹始终保持双轮向前。
    float max_diff = speed_set - min_wheel_speed;
    if (max_diff < 0.0f) {
        max_diff = 0.0f;
    }

    float duty_cmd = fclip(Duty_dS, -max_diff, max_diff);
    if (fabsf(line_error_filtered) > 1.2f && duty_limit < 6.4f) {
        duty_limit = 6.4f;
    }
    duty_cmd = fclip(duty_cmd, -duty_limit, duty_limit);

    // 削掉传感器/陀螺仪微小偏置造成的持续轻微转向。
    if (fabsf(duty_cmd) < 0.08f) {
        duty_cmd = 0.0f;
    }
    Duty_dS_applied = duty_cmd;

    // R = speed + 补偿，L = speed - 补偿；不允许负向轮速。
    float speed_set_diff_R_target =
        fclip(speed_set + duty_cmd, 0.0f, speed_set_max);
    float speed_set_diff_L_target =
        fclip(speed_set - duty_cmd, 0.0f, speed_set_max);

    float speed_diff_step_R =
        fclip(speed_set_diff_R_target - speed_set_diff_R_last,
              -speed_change_rate_max,
              speed_change_rate_max);
    speed_set_diff_R = speed_set_diff_R_last + speed_diff_step_R;

    float speed_diff_step_L =
        fclip(speed_set_diff_L_target - speed_set_diff_L_last,
              -speed_change_rate_max,
              speed_change_rate_max);
    speed_set_diff_L = speed_set_diff_L_last + speed_diff_step_L;

    speed_set_diff_R_last = speed_set_diff_R;
    speed_set_diff_L_last = speed_set_diff_L;

    motor_set_R(speed_set_diff_R);
    motor_set_L(speed_set_diff_L);
}


// =========================================================================
// 第六部分：终极控制大循环 (挂载在 5ms 中断里)
// =========================================================================
void tracking_control_loop(void)
{
    // =========================================================
    // 1. 感知层：始终睁眼，获取最新鲜的物理世界数据
    // =========================================================
    // 必须放在所有 return 的最前面！无论车走不走，底层滤波器都在收敛！
    adc_capture();
    calculate_line_error();

    // =========================================================
    // 2. 启停拦截层：处理发车延时与强制停机
    // =========================================================
    if (start_delay > 0) {
        start_delay--;
    }

    if (battery_3s_update_5ms(
            battery_voltage,
            (uint8_t)(motor_enable != 0 &&
                      task_number != TASK_H_STATIC_BALL_TRANSFER))) {
        motor_enable = 0;
        run_state = 0;
        beep_set_time(500);
    }

    if (!battery_3s_motor_allowed()) {
        motor_enable = 0;
        run_state = 0;
    }

    // 如果未使能、Task3/禁用任务要求静止、或还在倒计时，电机强行给0并退出。
    if (motor_enable == 0 ||
        task_number == TASK_H_STATIC_BALL_TRANSFER ||
        (task_number >= TASK_H_DISABLED_FIRST &&
         task_number <= TASK_H_DISABLED_LAST) ||
        start_delay > 0) {
        Duty_dS_applied = 0.0f;
        motor_set_R(0);
        motor_set_L(0);
        return; // 拦截：本回合不进行运动计算，但上面的“眼睛”已经更新了！
    }

#if PWM_EQUAL_TEST_ENABLE
    // 开环测试：左右轮同方向、同占空比，不走PID。
    Duty_dS = 0.0f;
    Duty_dS_applied = 0.0f;

    gpio_set_level(DIR_R, 0);
    gpio_set_level(DIR_L, 0);
    pwm_set_duty(PWM_R, battery_3s_compensate_motor_duty(PWM_EQUAL_TEST_DUTY, battery_voltage));
    pwm_set_duty(PWM_L, battery_3s_compensate_motor_duty(PWM_EQUAL_TEST_DUTY, battery_voltage));
    return;
#endif

    // Task2独有：按键发车后计时，约300°开始减速，仅在末段识别A点宽线。
    // Task1/Task9/Task10持续环线测试，只由KEY4或安全保护停止。
    if (task_number == TASK_H_ONE_LAP_STOP &&
        run_state != 0 && motor_enable != 0) {
        if (h_task2_update_5ms()) {
            return;
        }
    }

    // =========================================================
    // 3. 算法层：外环角度 + 内环角速度串级 PID 计算
    // =========================================================
    float line_error_ctrl;
    if (task_uses_enhanced_h_tracking()) {
        // Task1/Task2/Task9：H题直线与0.5 m半圆弧统一循迹。
        line_error_ctrl = tracking_shape_line_error(curve_tracking_control())
                        * CURVE_CONTROL_GAIN;
    } else {
        // Task10：保留基础误差整形，用于电脑在线调节外环PID。
        curve_tracking_reset();
        line_error_ctrl = standard_tracking_shape_line_error(line_error_filtered);
    }
    // 巡线方向修正：反转外环输入符号，避免“压左线却继续向右偏”。
    aim_yaw_g = PID_Yaw_a(0, -line_error_ctrl);

    PID_Yaw_gyro(); // 计算内环陀螺仪角速度输出

    // =========================================================
    // 4. 执行层：最终输出给电机
    // =========================================================
    motor_control();

    // lap_control(); // 如果你有圈数控制，放在这里最后执行
}
