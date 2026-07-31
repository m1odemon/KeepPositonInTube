#include "servo.h"

static float servo_current_angle_1 = SERVO_MAX_ANGLE / 2.0f;
static float servo_current_angle_2 = SERVO_MAX_ANGLE / 2.0f;
static volatile float servo_target_angle_1 = SERVO_MAX_ANGLE / 2.0f;
static volatile float servo_target_angle_2 = SERVO_MAX_ANGLE / 2.0f;
static volatile uint8 servo_initialized_1 = 0U;
static volatile uint8 servo_initialized_2 = 0U;
static uint8 servo_soft_delay_ticks = 0U;

/* Task8 is intentionally a low-amplitude B4-only bench test. */
#define SERVO_TASK8_FORWARD_ANGLE       (80.0f)
#define SERVO_TASK8_REVERSE_ANGLE       (100.0f)
#define SERVO_TASK8_SWITCH_TICKS        (200U)  /* 200 x 5 ms = 1 s */

static volatile uint8 servo_task8_active = 0U;
static volatile uint8 servo_task8_forward = 1U;
static volatile uint32 servo_task8_ticks = 0U;

static float servo_limit_angle(float angle)
{
    if (angle < SERVO_MIN_ANGLE) {
        return SERVO_MIN_ANGLE;
    }

    if (angle > SERVO_MAX_ANGLE) {
        return SERVO_MAX_ANGLE;
    }

    return angle;
}

static uint32 servo_angle_to_pulse_us(float angle)
{
    float limited_angle = servo_limit_angle(angle);
    float pulse_us = (float)SERVO_MIN_PULSE_US
                   + (limited_angle / SERVO_MAX_ANGLE)
                   * (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    return (uint32)(pulse_us + 0.5f);
}

static float servo_pulse_us_to_angle(uint32 pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        pulse_us = SERVO_MIN_PULSE_US;
    }

    if (pulse_us > SERVO_MAX_PULSE_US) {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    return ((float)(pulse_us - SERVO_MIN_PULSE_US)
            * SERVO_MAX_ANGLE)
           / (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
}

static uint32 servo_pulse_us_to_duty(uint32 pulse_us)
{
    uint32 period_us = 1000000U / SERVO_PWM_FREQ;

    if (pulse_us < SERVO_MIN_PULSE_US) {
        pulse_us = SERVO_MIN_PULSE_US;
    }

    if (pulse_us > SERVO_MAX_PULSE_US) {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    return (uint32)(((uint64)pulse_us * PWM_DUTY_MAX + period_us / 2U)
                    / period_us);
}

static uint32 servo_limit_pulse_us(uint32 pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        return SERVO_MIN_PULSE_US;
    }

    if (pulse_us > SERVO_MAX_PULSE_US) {
        return SERVO_MAX_PULSE_US;
    }

    return pulse_us;
}

static uint8 servo_channel_is_valid(servo_channel_enum channel)
{
    return (uint8)(channel == SERVO_CHANNEL_1 || channel == SERVO_CHANNEL_2);
}

static pwm_channel_enum servo_channel_to_pwm(servo_channel_enum channel)
{
    if (channel == SERVO_CHANNEL_2) {
        return SERVO_PWM_PIN_2;
    }

    return SERVO_PWM_PIN;
}

static float *servo_get_current_angle(servo_channel_enum channel)
{
    if (channel == SERVO_CHANNEL_2) {
        return &servo_current_angle_2;
    }

    return &servo_current_angle_1;
}

static volatile float *servo_get_target_angle(servo_channel_enum channel)
{
    if (channel == SERVO_CHANNEL_2) {
        return &servo_target_angle_2;
    }

    return &servo_target_angle_1;
}

static volatile uint8 *servo_get_initialized(servo_channel_enum channel)
{
    if (channel == SERVO_CHANNEL_2) {
        return &servo_initialized_2;
    }

    return &servo_initialized_1;
}

static void servo_write_angle_channel(servo_channel_enum channel, float angle)
{
    pwm_set_duty(servo_channel_to_pwm(channel),
                 servo_pulse_us_to_duty(servo_angle_to_pulse_us(angle)));
}

static void servo_write_pulse_us_channel(servo_channel_enum channel, uint32 pulse_us)
{
    pwm_set_duty(servo_channel_to_pwm(channel),
                 servo_pulse_us_to_duty(servo_limit_pulse_us(pulse_us)));
}

static void servo_move_one_step(servo_channel_enum channel)
{
    float *current_angle = servo_get_current_angle(channel);
    float target_angle = *servo_get_target_angle(channel);

    if (*current_angle < target_angle) {
        *current_angle += SERVO_SOFT_STEP_DEG;
        if (*current_angle > target_angle) {
            *current_angle = target_angle;
        }

        servo_write_angle_channel(channel, *current_angle);
    }
    else if (*current_angle > target_angle) {
        *current_angle -= SERVO_SOFT_STEP_DEG;
        if (*current_angle < target_angle) {
            *current_angle = target_angle;
        }

        servo_write_angle_channel(channel, *current_angle);
    }
}

uint8 servo_init(void)
{
    return servo_init_channel(SERVO_CHANNEL_1);
}

uint8 servo_init_channel(servo_channel_enum channel)
{
    float *current_angle;
    volatile uint8 *initialized;

    if (!servo_channel_is_valid(channel)) {
        return 0U;
    }

    current_angle = servo_get_current_angle(channel);
    initialized = servo_get_initialized(channel);
    pwm_init(servo_channel_to_pwm(channel), SERVO_PWM_FREQ, 0U);
    *current_angle = SERVO_MAX_ANGLE / 2.0f;
    *servo_get_target_angle(channel) = *current_angle;
    servo_write_angle_channel(channel, *current_angle);
    *initialized = 1U;
    return 1U;
}

uint8 servo_set_angle(float angle)
{
    return servo_set_angle_channel(SERVO_CHANNEL_1, angle);
}

uint8 servo_set_angle_channel(servo_channel_enum channel, float angle)
{
    if (!servo_channel_is_valid(channel) || !*servo_get_initialized(channel)) {
        return 0U;
    }

    *servo_get_target_angle(channel) = servo_limit_angle(angle);
    return 1U;
}

uint8 servo_set_pulse_us(uint32 pulse_us)
{
    return servo_set_pulse_us_channel(SERVO_CHANNEL_1, pulse_us);
}

uint8 servo_set_pulse_us_channel(servo_channel_enum channel, uint32 pulse_us)
{
    return servo_set_angle_channel(channel, servo_pulse_us_to_angle(pulse_us));
}

uint8 servo_apply_pulse_us_now(uint32 pulse_us)
{
    return servo_apply_pulse_us_now_channel(SERVO_CHANNEL_1, pulse_us);
}

uint8 servo_apply_pulse_us_now_channel(servo_channel_enum channel, uint32 pulse_us)
{
    float pulse_angle;
    float *current_angle;
    volatile float *target_angle;

    if (!servo_channel_is_valid(channel) || !*servo_get_initialized(channel)) {
        return 0U;
    }

    pulse_us = servo_limit_pulse_us(pulse_us);
    pulse_angle = servo_pulse_us_to_angle(pulse_us);
    current_angle = servo_get_current_angle(channel);
    target_angle = servo_get_target_angle(channel);

    *current_angle = pulse_angle;
    *target_angle = pulse_angle;
    servo_write_pulse_us_channel(channel, pulse_us);
    return 1U;
}

uint8 servo_task8_start(void)
{
    if (servo_init_channel(SERVO_CHANNEL_1) == 0U) {
        return 0U;
    }

    servo_task8_forward = 1U;
    servo_task8_ticks = 0U;
    servo_task8_active = 1U;
    return servo_set_angle(SERVO_TASK8_FORWARD_ANGLE);
}

void servo_task8_stop(void)
{
    servo_task8_active = 0U;
    servo_task8_ticks = 0U;

    if (*servo_get_initialized(SERVO_CHANNEL_1)) {
        (void)servo_set_angle(SERVO_MAX_ANGLE / 2.0f);
    }
}

void servo_task8_update_5ms(void)
{
    if (!servo_task8_active) {
        return;
    }

    servo_task8_ticks++;
    if (servo_task8_ticks < SERVO_TASK8_SWITCH_TICKS) {
        return;
    }

    servo_task8_ticks = 0U;
    servo_task8_forward = (uint8)!servo_task8_forward;
    (void)servo_set_angle(servo_task8_forward
                         ? SERVO_TASK8_FORWARD_ANGLE
                         : SERVO_TASK8_REVERSE_ANGLE);
}

void servo_update_5ms(void)
{
    if (++servo_soft_delay_ticks < SERVO_SOFT_DELAY_TICKS) {
        return;
    }

    servo_soft_delay_ticks = 0U;

    if (*servo_get_initialized(SERVO_CHANNEL_1)) {
        servo_move_one_step(SERVO_CHANNEL_1);
    }

    if (*servo_get_initialized(SERVO_CHANNEL_2)) {
        servo_move_one_step(SERVO_CHANNEL_2);
    }
}
