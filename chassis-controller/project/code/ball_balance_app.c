#include "ball_balance_app.h"

#include <stddef.h>

#include "servo.h"

static void ball_balance_app_apply_output(ball_balance_app_t *app, uint32_t now_us)
{
    ball_control_state_enum state = app->controller.state;

    if (state == BALL_CONTROL_TRACKING || state == BALL_CONTROL_HOLD)
    {
        ball_actuator_command_angle(
            &app->actuator,
            ball_control_get_target_tube_angle_deg(&app->controller),
            now_us);
    }
    else
    {
        ball_actuator_force_safe(&app->actuator, now_us);
    }

    (void)servo_apply_pulse_us_now_channel(SERVO_CHANNEL_1,
                                            ball_actuator_get_pulse_us(&app->actuator));
}

bool ball_balance_app_init(ball_balance_app_t *app,
                           const ball_control_config_t *config,
                           uint32_t now_us)
{
    return ball_balance_app_init_with_config(app, config, NULL, now_us);
}

bool ball_balance_app_init_with_config(
    ball_balance_app_t *app,
    const ball_control_config_t *control_config,
    const ball_actuator_config_t *actuator_config,
    uint32_t now_us)
{
    if (app == NULL)
    {
        return false;
    }

    ball_control_init(&app->controller, control_config);
    if (!ball_actuator_init(&app->actuator, actuator_config, now_us))
    {
        app->initialized = false;
        return false;
    }
    ball_vision_link_init(&app->vision_link);

    if (servo_init_channel(SERVO_CHANNEL_1) == 0U)
    {
        app->initialized = false;
        return false;
    }

    app->initialized = true;
    ball_control_tick(&app->controller, now_us);
    ball_balance_app_apply_output(app, now_us);
    return true;
}

void ball_balance_app_set_target_mm(ball_balance_app_t *app, float target_position_mm)
{
    if (app != NULL && app->initialized)
    {
        ball_control_set_target_mm(&app->controller, target_position_mm);
    }
}

void ball_balance_app_set_vehicle_acceleration_mm_s2(ball_balance_app_t *app,
                                                      float acceleration_mm_s2)
{
    if (app != NULL && app->initialized)
    {
        ball_control_set_vehicle_acceleration_mm_s2(&app->controller,
                                                    acceleration_mm_s2);
    }
}

void ball_balance_app_set_enable(ball_balance_app_t *app, bool enable, uint32_t now_us)
{
    if (app != NULL && app->initialized)
    {
        ball_control_request_enable(&app->controller, enable, now_us);
        ball_balance_app_apply_output(app, now_us);
    }
}

bool ball_balance_app_reset_fault(ball_balance_app_t *app, uint32_t now_us)
{
    if (app == NULL || !app->initialized)
    {
        return false;
    }

    if (!ball_control_reset_fault(&app->controller))
    {
        return false;
    }

    ball_control_tick(&app->controller, now_us);
    ball_balance_app_apply_output(app, now_us);
    return true;
}

void ball_balance_app_emergency_stop(ball_balance_app_t *app, uint32_t now_us)
{
    if (app != NULL && app->initialized)
    {
        ball_control_emergency_stop(&app->controller, now_us);
        ball_balance_app_apply_output(app, now_us);
    }
}

void ball_balance_app_rx_uart_byte_from_isr(ball_balance_app_t *app,
                                             uint8_t byte,
                                             uint32_t receive_timestamp_us)
{
    if (app != NULL && app->initialized)
    {
        ball_vision_link_rx_byte_from_isr(&app->vision_link,
                                           byte,
                                           receive_timestamp_us);
    }
}

void ball_balance_app_tick_5ms(ball_balance_app_t *app, uint32_t now_us)
{
    ball_vision_frame_t frame;

    if (app == NULL || !app->initialized)
    {
        return;
    }

    if (ball_vision_link_take_latest(&app->vision_link, &frame))
    {
        (void)ball_control_consume_measurement(&app->controller,
                                               &frame.measurement,
                                               frame.receive_timestamp_us);
    }

    ball_control_tick(&app->controller, now_us);
    ball_balance_app_apply_output(app, now_us);
}

const ball_control_t *ball_balance_app_get_controller(const ball_balance_app_t *app)
{
    return (app != NULL) ? &app->controller : NULL;
}

const ball_actuator_t *ball_balance_app_get_actuator(const ball_balance_app_t *app)
{
    return (app != NULL) ? &app->actuator : NULL;
}

const ball_vision_link_t *ball_balance_app_get_vision_link(const ball_balance_app_t *app)
{
    return (app != NULL) ? &app->vision_link : NULL;
}
