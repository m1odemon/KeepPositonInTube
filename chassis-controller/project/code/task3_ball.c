#include "task3_ball.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "ball_actuator.h"
#include "servo.h"

#define TASK3_TICK_US            (5000U)
#define TASK3_TICK_MS            (5U)
#define TASK3_PLUS_TARGET_MM     (50.0f)
#define TASK3_MINUS_TARGET_MM    (-50.0f)
#define TASK3_TIME_LIMIT_MS      (5000U)

static ball_control_t task3_controller;
static ball_actuator_t task3_actuator;
static ball_control_measurement_t task3_pending_measurement;

static volatile task3_ball_state_enum task3_state = TASK3_BALL_READY;
static volatile uint32_t task3_now_us = 0U;
static volatile uint32_t task3_elapsed_ms = 0U;
static volatile uint8_t task3_active = 0U;
static volatile uint8_t task3_measurement_pending = 0U;
static volatile uint8_t task3_has_position = 0U;
static volatile uint8_t task3_finished = 0U;
static volatile uint8_t task3_overtime = 0U;

static void task3_apply_safe_output(void)
{
    ball_actuator_force_safe(&task3_actuator, task3_now_us);
    (void)servo_apply_pulse_us_now_channel(
        SERVO_CHANNEL_1,
        ball_actuator_get_pulse_us(&task3_actuator));
}

static void task3_apply_controller_output(void)
{
    ball_control_state_enum control_state = task3_controller.state;

    if (control_state == BALL_CONTROL_TRACKING ||
        control_state == BALL_CONTROL_HOLD)
    {
        ball_actuator_command_angle(
            &task3_actuator,
            ball_control_get_target_tube_angle_deg(&task3_controller),
            task3_now_us);
        (void)servo_apply_pulse_us_now_channel(
            SERVO_CHANNEL_1,
            ball_actuator_get_pulse_us(&task3_actuator));
    }
    else
    {
        task3_apply_safe_output();
    }
}

static uint8_t task3_prepare_control(void)
{
    ball_control_config_t control_config;
    ball_actuator_config_t actuator_config;

    ball_control_config_defaults(&control_config);
    ball_actuator_config_defaults(&actuator_config);
    ball_control_init(&task3_controller, &control_config);

    if (!ball_actuator_init(&task3_actuator,
                            &actuator_config,
                            task3_now_us))
    {
        return 0U;
    }

    if (servo_init_channel(SERVO_CHANNEL_1) == 0U)
    {
        return 0U;
    }

    task3_apply_safe_output();
    return 1U;
}

void task3_ball_init(void)
{
    memset(&task3_controller, 0, sizeof(task3_controller));
    memset(&task3_actuator, 0, sizeof(task3_actuator));
    memset(&task3_pending_measurement, 0, sizeof(task3_pending_measurement));
    task3_state = TASK3_BALL_READY;
    task3_elapsed_ms = 0U;
    task3_active = 0U;
    task3_measurement_pending = 0U;
    task3_has_position = 0U;
    task3_finished = 0U;
    task3_overtime = 0U;
}

uint8_t task3_ball_start(void)
{
    task3_active = 0U;
    task3_state = TASK3_BALL_READY;
    task3_elapsed_ms = 0U;
    task3_measurement_pending = 0U;
    task3_has_position = 0U;
    task3_finished = 0U;
    task3_overtime = 0U;

    if (!task3_prepare_control())
    {
        task3_state = TASK3_BALL_FAULT;
        return 0U;
    }

    task3_state = TASK3_BALL_WAIT_SENSOR;
    task3_active = 1U;
    return 1U;
}

void task3_ball_stop(void)
{
    task3_active = 0U;
    task3_finished = 0U;
    task3_measurement_pending = 0U;

    if (task3_state != TASK3_BALL_READY)
    {
        ball_control_request_enable(&task3_controller, false, task3_now_us);
        task3_apply_safe_output();
    }

    task3_state = TASK3_BALL_READY;
}

void task3_ball_submit_measurement(const ball_control_measurement_t *measurement)
{
    if (measurement == NULL || !task3_active)
    {
        return;
    }

    /*
     * 当前没有通信调用者。未来接入时，UART 解析器应先在自己的缓冲区发布
     * 完整帧，再从 5 ms 任务中调用本函数，避免在中断中运行浮点控制。
     */
    task3_pending_measurement = *measurement;
    task3_measurement_pending = 1U;
}

void task3_ball_update_5ms(void)
{
    ball_control_measurement_t measurement;
    uint8_t measurement_ready = 0U;

    task3_now_us += TASK3_TICK_US;

    if (!task3_active)
    {
        return;
    }

    if (!task3_finished)
    {
        if (task3_elapsed_ms <= UINT32_MAX - TASK3_TICK_MS)
        {
            task3_elapsed_ms += TASK3_TICK_MS;
        }
        else
        {
            task3_elapsed_ms = UINT32_MAX;
        }

        if (task3_elapsed_ms > TASK3_TIME_LIMIT_MS)
        {
            task3_overtime = 1U;
        }
    }

    if (task3_measurement_pending)
    {
        measurement = task3_pending_measurement;
        task3_measurement_pending = 0U;
        measurement_ready = 1U;
        task3_has_position = 1U;
    }

    if (task3_state == TASK3_BALL_WAIT_SENSOR)
    {
        if (!measurement_ready)
        {
            task3_apply_safe_output();
            return;
        }

        ball_control_set_target_mm(&task3_controller,
                                   TASK3_PLUS_TARGET_MM);
        ball_control_request_enable(&task3_controller, true, task3_now_us);
        (void)ball_control_consume_measurement(&task3_controller,
                                               &measurement,
                                               task3_now_us);
        task3_state = TASK3_BALL_TO_PLUS_50;
    }
    else if (measurement_ready)
    {
        (void)ball_control_consume_measurement(&task3_controller,
                                               &measurement,
                                               task3_now_us);
    }

    ball_control_tick(&task3_controller, task3_now_us);
    if (task3_controller.state == BALL_CONTROL_FAULT_LATCHED)
    {
        task3_state = TASK3_BALL_FAULT;
        task3_finished = 1U;
        task3_apply_safe_output();
        return;
    }

    if (task3_state == TASK3_BALL_TO_PLUS_50 &&
        task3_controller.state == BALL_CONTROL_HOLD)
    {
        ball_control_set_target_mm(&task3_controller,
                                   TASK3_MINUS_TARGET_MM);
        task3_state = TASK3_BALL_TO_MINUS_50;
    }
    else if (task3_state == TASK3_BALL_TO_MINUS_50 &&
             task3_controller.state == BALL_CONTROL_HOLD)
    {
        task3_state = TASK3_BALL_DONE_HOLD;
        task3_finished = 1U;
    }

    task3_apply_controller_output();
}

task3_ball_state_enum task3_ball_get_state(void)
{
    return task3_state;
}

ball_control_fault_enum task3_ball_get_fault(void)
{
    return task3_controller.fault;
}

uint32_t task3_ball_get_elapsed_ms(void)
{
    return task3_elapsed_ms;
}

float task3_ball_get_target_mm(void)
{
    if (task3_state == TASK3_BALL_TO_MINUS_50 ||
        task3_state == TASK3_BALL_DONE_HOLD)
    {
        return TASK3_MINUS_TARGET_MM;
    }
    if (task3_state == TASK3_BALL_TO_PLUS_50)
    {
        return TASK3_PLUS_TARGET_MM;
    }
    return 0.0f;
}

float task3_ball_get_position_mm(void)
{
    return task3_has_position ? task3_controller.filtered_position_mm : 0.0f;
}

uint8_t task3_ball_has_measurement(void)
{
    return task3_has_position;
}

uint8_t task3_ball_is_finished(void)
{
    return task3_finished;
}

uint8_t task3_ball_is_overtime(void)
{
    return task3_overtime;
}
