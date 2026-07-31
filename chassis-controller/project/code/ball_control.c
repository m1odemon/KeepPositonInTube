#include "ball_control.h"

#include <math.h>
#include <string.h>

static float ball_control_clip(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static bool ball_control_sequence_is_newer(uint32_t sequence, uint32_t previous_sequence)
{
    uint32_t difference = sequence - previous_sequence;
    return difference != 0U && difference < 0x80000000UL;
}

static float ball_control_filter_alpha(float dt_s, float tau_s)
{
    if (tau_s <= 0.0f)
    {
        return 1.0f;
    }
    return 1.0f - expf(-dt_s / tau_s);
}

static void ball_control_clear_motion_history(ball_control_t *controller)
{
    controller->filtered_velocity_mm_s = 0.0f;
    controller->filtered_acceleration_mm_s2 = 0.0f;
    controller->velocity_valid = false;
    controller->acceleration_valid = false;
    controller->settle_elapsed_us = 0U;
}

static void ball_control_clear_control_memory(ball_control_t *controller)
{
    controller->position_integral_mm_s = 0.0f;
    controller->target_velocity_mm_s = 0.0f;
    controller->target_tube_angle_deg = 0.0f;
    controller->settle_elapsed_us = 0U;
}

static void ball_control_reset_estimator(ball_control_t *controller,
                                         const ball_control_measurement_t *measurement)
{
    controller->filtered_position_mm = measurement->position_mm;
    controller->last_raw_position_mm = measurement->position_mm;
    controller->last_capture_timestamp_us = measurement->capture_timestamp_us;
    controller->last_sequence = measurement->sequence;
    controller->estimator_initialized = true;
    controller->sequence_valid = true;
    ball_control_clear_motion_history(controller);
}

static void ball_control_apply_requested_angle(ball_control_t *controller,
                                               float requested_angle_deg,
                                               float angle_limit_deg,
                                               uint32_t now_us)
{
    float absolute_limit_deg = fabsf(angle_limit_deg);
    float clipped_request = ball_control_clip(requested_angle_deg,
                                              -absolute_limit_deg,
                                              absolute_limit_deg);

    if (!controller->output_timestamp_valid)
    {
        controller->target_tube_angle_deg = clipped_request;
        controller->last_output_timestamp_us = now_us;
        controller->output_timestamp_valid = true;
        return;
    }

    uint32_t elapsed_us = now_us - controller->last_output_timestamp_us;
    if (controller->config.tube_angle_rate_limit_deg_s > 0.0f && elapsed_us > 0U)
    {
        float max_step = controller->config.tube_angle_rate_limit_deg_s
                       * ((float)elapsed_us / 1000000.0f);
        controller->target_tube_angle_deg =
            ball_control_clip(clipped_request,
                              controller->target_tube_angle_deg - max_step,
                              controller->target_tube_angle_deg + max_step);
    }
    else
    {
        controller->target_tube_angle_deg = clipped_request;
    }

    controller->last_output_timestamp_us = now_us;
}

static void ball_control_apply_safe_angle(ball_control_t *controller, uint32_t now_us)
{
    controller->target_velocity_mm_s = 0.0f;
    controller->target_tube_angle_deg = 0.0f;
    controller->last_output_timestamp_us = now_us;
    controller->output_timestamp_valid = true;
}

static void ball_control_enter_fault(ball_control_t *controller,
                                     ball_control_fault_enum fault,
                                     uint32_t now_us)
{
    controller->state = BALL_CONTROL_FAULT_LATCHED;
    controller->fault = fault;
    controller->enable_requested = false;
    controller->arming_frame_count = 0U;
    controller->arming_started_timestamp_valid = false;
    ball_control_clear_motion_history(controller);
    ball_control_clear_control_memory(controller);
    ball_control_apply_safe_angle(controller, now_us);
}

static void ball_control_update_hold_state(ball_control_t *controller,
                                           float position_error_mm,
                                           float dt_s)
{
    float absolute_error = fabsf(position_error_mm);
    float absolute_velocity = fabsf(controller->filtered_velocity_mm_s);

    if (controller->state == BALL_CONTROL_HOLD)
    {
        if (absolute_error > controller->config.exit_hold_position_error_mm)
        {
            controller->state = BALL_CONTROL_TRACKING;
            controller->settle_elapsed_us = 0U;
        }
        return;
    }

    if (absolute_error <= controller->config.position_tolerance_mm &&
        absolute_velocity <= controller->config.velocity_tolerance_mm_s)
    {
        controller->settle_elapsed_us += (uint32_t)(dt_s * 1000000.0f + 0.5f);
        if (controller->settle_elapsed_us >= (uint32_t)controller->config.settle_time_ms * 1000U)
        {
            controller->state = BALL_CONTROL_HOLD;
        }
    }
    else
    {
        controller->settle_elapsed_us = 0U;
    }
}

static float ball_control_update_target_velocity(ball_control_t *controller,
                                                 float position_error_mm,
                                                 float dt_s,
                                                 float speed_limit_mm_s)
{
    float proportional_speed = controller->config.position_kp_per_s
                             * position_error_mm;
    float integral_gain = controller->config.position_ki_per_s2;
    float absolute_limit = fabsf(speed_limit_mm_s);

    if (integral_gain <= 0.0f)
    {
        controller->position_integral_mm_s = 0.0f;
    }
    else if (controller->config.position_integral_zone_mm <= 0.0f ||
             fabsf(position_error_mm) <= controller->config.position_integral_zone_mm)
    {
        float integral_limit = fabsf(controller->config.position_integral_limit_mm_s);
        float candidate_integral = controller->position_integral_mm_s
                                 + position_error_mm * dt_s;
        candidate_integral = ball_control_clip(candidate_integral,
                                               -integral_limit,
                                               integral_limit);

        float candidate_speed = proportional_speed
                              + integral_gain * candidate_integral;
        bool output_not_saturated = fabsf(candidate_speed) <= absolute_limit;
        bool unwinding_positive_saturation =
            candidate_speed > absolute_limit && position_error_mm < 0.0f;
        bool unwinding_negative_saturation =
            candidate_speed < -absolute_limit && position_error_mm > 0.0f;

        if (output_not_saturated ||
            unwinding_positive_saturation ||
            unwinding_negative_saturation)
        {
            controller->position_integral_mm_s = candidate_integral;
        }
    }
    else
    {
        /* Integral separation avoids windup during large setpoint changes. */
        controller->position_integral_mm_s = 0.0f;
    }

    return ball_control_clip(proportional_speed
                             + integral_gain * controller->position_integral_mm_s,
                             -absolute_limit,
                             absolute_limit);
}

static void ball_control_update_output(ball_control_t *controller,
                                       float dt_s,
                                       uint32_t receive_timestamp_us)
{
    float position_error_mm = controller->target_position_mm - controller->filtered_position_mm;
    ball_control_update_hold_state(controller, position_error_mm, dt_s);

    float speed_limit_mm_s = controller->config.target_speed_limit_mm_s;
    float angle_limit_deg = controller->config.tube_angle_limit_deg;
    if (controller->state == BALL_CONTROL_HOLD)
    {
        speed_limit_mm_s = controller->config.hold_target_speed_limit_mm_s;
        angle_limit_deg = controller->config.hold_tube_angle_limit_deg;
    }

    controller->target_velocity_mm_s =
        ball_control_update_target_velocity(controller,
                                            position_error_mm,
                                            dt_s,
                                            speed_limit_mm_s);

    float velocity_error_mm_s = controller->target_velocity_mm_s
                              - controller->filtered_velocity_mm_s;
    float requested_angle_deg =
        controller->config.velocity_kp_deg_s_per_mm * velocity_error_mm_s
        - controller->config.velocity_kd_deg_s2_per_mm
          * controller->filtered_acceleration_mm_s2
        + controller->config.vehicle_accel_feedforward_deg_s2_per_mm
          * controller->vehicle_acceleration_mm_s2;
    requested_angle_deg = ball_control_clip(requested_angle_deg,
                                            -fabsf(angle_limit_deg),
                                            fabsf(angle_limit_deg));

    if (controller->state == BALL_CONTROL_TRACKING &&
        fabsf(position_error_mm) > controller->config.exit_hold_position_error_mm &&
        requested_angle_deg != 0.0f)
    {
        float minimum_angle = (requested_angle_deg > 0.0f)
                            ? controller->config.min_effective_angle_pos_deg
                            : controller->config.min_effective_angle_neg_deg;
        minimum_angle = fabsf(minimum_angle);
        if (fabsf(requested_angle_deg) < minimum_angle)
        {
            requested_angle_deg =
                (requested_angle_deg > 0.0f) ? minimum_angle : -minimum_angle;
        }
    }

    ball_control_apply_requested_angle(controller,
                                       requested_angle_deg,
                                       angle_limit_deg,
                                       receive_timestamp_us);
}

void ball_control_config_defaults(ball_control_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->position_min_mm = -120.0f;
    config->position_max_mm = 120.0f;
    config->position_filter_tau_s = 0.040f;
    config->velocity_filter_tau_s = 0.070f;
    config->acceleration_filter_tau_s = 0.120f;
    config->min_confidence = 0.70f;
    config->max_valid_speed_mm_s = 500.0f;
    config->max_position_innovation_mm = 12.0f;
    config->position_kp_per_s = 1.20f;
    config->position_ki_per_s2 = 0.0f;
    config->position_integral_zone_mm = 10.0f;
    config->position_integral_limit_mm_s = 20.0f;
    config->target_speed_limit_mm_s = 120.0f;
    config->hold_target_speed_limit_mm_s = 30.0f;
    config->velocity_kp_deg_s_per_mm = 0.035f;
    config->velocity_kd_deg_s2_per_mm = 0.0f;
    config->tube_angle_limit_deg = 5.0f;
    config->hold_tube_angle_limit_deg = 2.0f;
    config->min_effective_angle_pos_deg = 0.0f;
    config->min_effective_angle_neg_deg = 0.0f;
    config->tube_angle_rate_limit_deg_s = 50.0f;
    config->vehicle_accel_feedforward_deg_s2_per_mm = 0.0f;
    config->position_tolerance_mm = 2.0f;
    config->exit_hold_position_error_mm = 4.0f;
    config->velocity_tolerance_mm_s = 8.0f;
    config->dt_min_s = 0.005f;
    config->dt_max_s = 0.150f;
    config->vision_timeout_ms = 180U;
    config->settle_time_ms = 300U;
    config->arming_valid_frames = 3U;
}

void ball_control_init(ball_control_t *controller, const ball_control_config_t *config)
{
    ball_control_config_t default_config;

    if (controller == NULL)
    {
        return;
    }

    if (config == NULL)
    {
        ball_control_config_defaults(&default_config);
        config = &default_config;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = *config;

    if (controller->config.position_max_mm <= controller->config.position_min_mm)
    {
        controller->config.position_max_mm = controller->config.position_min_mm + 1.0f;
    }
    if (controller->config.target_speed_limit_mm_s <= 0.0f)
    {
        controller->config.target_speed_limit_mm_s = 1.0f;
    }
    if (controller->config.hold_target_speed_limit_mm_s <= 0.0f ||
        controller->config.hold_target_speed_limit_mm_s >
            controller->config.target_speed_limit_mm_s)
    {
        controller->config.hold_target_speed_limit_mm_s =
            controller->config.target_speed_limit_mm_s;
    }
    if (controller->config.tube_angle_limit_deg <= 0.0f)
    {
        controller->config.tube_angle_limit_deg = 1.0f;
    }
    if (controller->config.hold_tube_angle_limit_deg <= 0.0f ||
        controller->config.hold_tube_angle_limit_deg >
            controller->config.tube_angle_limit_deg)
    {
        controller->config.hold_tube_angle_limit_deg =
            controller->config.tube_angle_limit_deg;
    }
    if (controller->config.position_integral_zone_mm < 0.0f)
    {
        controller->config.position_integral_zone_mm =
            -controller->config.position_integral_zone_mm;
    }
    if (controller->config.position_integral_limit_mm_s < 0.0f)
    {
        controller->config.position_integral_limit_mm_s =
            -controller->config.position_integral_limit_mm_s;
    }
    if (controller->config.arming_valid_frames == 0U)
    {
        controller->config.arming_valid_frames = 1U;
    }
    if (controller->config.exit_hold_position_error_mm < controller->config.position_tolerance_mm)
    {
        controller->config.exit_hold_position_error_mm = controller->config.position_tolerance_mm;
    }

    controller->state = BALL_CONTROL_IDLE;
    controller->fault = BALL_CONTROL_FAULT_NONE;
    controller->target_position_mm = ball_control_clip(0.0f,
                                                        controller->config.position_min_mm,
                                                        controller->config.position_max_mm);
    controller->target_tube_angle_deg = 0.0f;
}

void ball_control_set_target_mm(ball_control_t *controller, float target_position_mm)
{
    if (controller == NULL || !isfinite(target_position_mm))
    {
        return;
    }

    controller->target_position_mm = ball_control_clip(target_position_mm,
                                                        controller->config.position_min_mm,
                                                        controller->config.position_max_mm);
    controller->position_integral_mm_s = 0.0f;
    controller->target_velocity_mm_s = 0.0f;
    controller->settle_elapsed_us = 0U;
    if (controller->state == BALL_CONTROL_HOLD)
    {
        controller->state = BALL_CONTROL_TRACKING;
    }
}

void ball_control_set_vehicle_acceleration_mm_s2(ball_control_t *controller,
                                                  float acceleration_mm_s2)
{
    if (controller != NULL && isfinite(acceleration_mm_s2))
    {
        controller->vehicle_acceleration_mm_s2 = acceleration_mm_s2;
    }
}

void ball_control_request_enable(ball_control_t *controller, bool enable, uint32_t now_us)
{
    if (controller == NULL || controller->state == BALL_CONTROL_FAULT_LATCHED)
    {
        return;
    }

    controller->enable_requested = enable;
    controller->arming_frame_count = 0U;
    controller->settle_elapsed_us = 0U;

    if (enable)
    {
        controller->state = BALL_CONTROL_ARMING;
        controller->estimator_initialized = false;
        controller->sequence_valid = false;
        controller->receive_timestamp_valid = false;
        controller->arming_started_timestamp_us = now_us;
        controller->arming_started_timestamp_valid = true;
        ball_control_clear_motion_history(controller);
        ball_control_clear_control_memory(controller);
        controller->last_output_timestamp_us = now_us;
        controller->output_timestamp_valid = true;
    }
    else
    {
        controller->state = BALL_CONTROL_IDLE;
        controller->fault = BALL_CONTROL_FAULT_NONE;
        controller->arming_started_timestamp_valid = false;
        ball_control_clear_motion_history(controller);
        ball_control_clear_control_memory(controller);
        ball_control_apply_safe_angle(controller, now_us);
    }
}

bool ball_control_reset_fault(ball_control_t *controller)
{
    if (controller == NULL || controller->state != BALL_CONTROL_FAULT_LATCHED)
    {
        return false;
    }

    controller->state = BALL_CONTROL_IDLE;
    controller->fault = BALL_CONTROL_FAULT_NONE;
    controller->enable_requested = false;
    controller->arming_frame_count = 0U;
    controller->estimator_initialized = false;
    controller->sequence_valid = false;
    controller->receive_timestamp_valid = false;
    controller->arming_started_timestamp_valid = false;
    controller->output_timestamp_valid = false;
    ball_control_clear_motion_history(controller);
    ball_control_clear_control_memory(controller);
    return true;
}

void ball_control_emergency_stop(ball_control_t *controller, uint32_t now_us)
{
    if (controller != NULL)
    {
        ball_control_enter_fault(controller, BALL_CONTROL_FAULT_EMERGENCY_STOP, now_us);
    }
}

bool ball_control_consume_measurement(ball_control_t *controller,
                                      const ball_control_measurement_t *measurement,
                                      uint32_t receive_timestamp_us)
{
    if (controller == NULL || measurement == NULL ||
        controller->state == BALL_CONTROL_FAULT_LATCHED)
    {
        return false;
    }

    if (!isfinite(measurement->position_mm) || !isfinite(measurement->confidence) ||
        measurement->confidence < controller->config.min_confidence)
    {
        return false;
    }

    if (measurement->position_mm < controller->config.position_min_mm ||
        measurement->position_mm > controller->config.position_max_mm)
    {
        ball_control_enter_fault(controller,
                                 BALL_CONTROL_FAULT_POSITION_HARD_LIMIT,
                                 receive_timestamp_us);
        return false;
    }

    if (controller->sequence_valid &&
        !ball_control_sequence_is_newer(measurement->sequence, controller->last_sequence))
    {
        return false;
    }

    if (!controller->estimator_initialized)
    {
        ball_control_reset_estimator(controller, measurement);
        controller->last_valid_receive_timestamp_us = receive_timestamp_us;
        controller->receive_timestamp_valid = true;
        if (controller->state == BALL_CONTROL_ARMING)
        {
            controller->arming_frame_count = 1U;
        }
        return true;
    }

    uint32_t capture_dt_us = measurement->capture_timestamp_us - controller->last_capture_timestamp_us;
    float dt_s = (float)capture_dt_us / 1000000.0f;
    if (dt_s < controller->config.dt_min_s || dt_s > controller->config.dt_max_s)
    {
        ball_control_reset_estimator(controller, measurement);
        ball_control_clear_control_memory(controller);
        ball_control_apply_safe_angle(controller, receive_timestamp_us);
        controller->last_valid_receive_timestamp_us = receive_timestamp_us;
        controller->receive_timestamp_valid = true;
        if (controller->state == BALL_CONTROL_ARMING)
        {
            controller->arming_frame_count = 1U;
        }
        return true;
    }

    float raw_speed_mm_s = (measurement->position_mm - controller->last_raw_position_mm) / dt_s;
    if (fabsf(raw_speed_mm_s) > controller->config.max_valid_speed_mm_s)
    {
        return false;
    }

    if (controller->velocity_valid)
    {
        float predicted_position_mm = controller->filtered_position_mm
                                    + controller->filtered_velocity_mm_s * dt_s;
        float innovation_limit_mm = controller->config.max_position_innovation_mm
                                  + controller->config.max_valid_speed_mm_s * dt_s;
        if (fabsf(measurement->position_mm - predicted_position_mm) > innovation_limit_mm)
        {
            return false;
        }
    }

    float previous_filtered_position_mm = controller->filtered_position_mm;
    float previous_filtered_velocity_mm_s = controller->filtered_velocity_mm_s;
    float alpha = ball_control_filter_alpha(dt_s, controller->config.position_filter_tau_s);
    float beta = ball_control_filter_alpha(dt_s, controller->config.velocity_filter_tau_s);
    controller->filtered_position_mm += alpha * (measurement->position_mm - controller->filtered_position_mm);
    float raw_velocity_mm_s = (controller->filtered_position_mm - previous_filtered_position_mm) / dt_s;
    controller->filtered_velocity_mm_s += beta * (raw_velocity_mm_s - controller->filtered_velocity_mm_s);

    if (controller->velocity_valid)
    {
        float raw_acceleration_mm_s2 =
            (controller->filtered_velocity_mm_s - previous_filtered_velocity_mm_s) / dt_s;
        float gamma = ball_control_filter_alpha(dt_s,
                                                controller->config.acceleration_filter_tau_s);
        if (controller->acceleration_valid)
        {
            controller->filtered_acceleration_mm_s2 +=
                gamma * (raw_acceleration_mm_s2
                         - controller->filtered_acceleration_mm_s2);
        }
        else
        {
            controller->filtered_acceleration_mm_s2 = raw_acceleration_mm_s2;
            controller->acceleration_valid = true;
        }
    }

    controller->last_raw_position_mm = measurement->position_mm;
    controller->last_capture_timestamp_us = measurement->capture_timestamp_us;
    controller->last_sequence = measurement->sequence;
    controller->sequence_valid = true;
    controller->velocity_valid = true;
    controller->last_valid_receive_timestamp_us = receive_timestamp_us;
    controller->receive_timestamp_valid = true;

    if (controller->state == BALL_CONTROL_ARMING)
    {
        if (controller->arming_frame_count < UINT8_MAX)
        {
            controller->arming_frame_count++;
        }
        if (controller->arming_frame_count >= controller->config.arming_valid_frames)
        {
            controller->state = BALL_CONTROL_TRACKING;
            controller->settle_elapsed_us = 0U;
        }
    }

    if (controller->state == BALL_CONTROL_TRACKING || controller->state == BALL_CONTROL_HOLD)
    {
        ball_control_update_output(controller, dt_s, receive_timestamp_us);
    }

    return true;
}

void ball_control_tick(ball_control_t *controller, uint32_t now_us)
{
    if (controller == NULL)
    {
        return;
    }

    if (controller->state == BALL_CONTROL_FAULT_LATCHED)
    {
        ball_control_apply_safe_angle(controller, now_us);
        return;
    }

    if (controller->state == BALL_CONTROL_IDLE)
    {
        ball_control_apply_safe_angle(controller, now_us);
        return;
    }

    uint32_t timeout_reference_us = controller->receive_timestamp_valid
                                  ? controller->last_valid_receive_timestamp_us
                                  : controller->arming_started_timestamp_us;
    if (controller->arming_started_timestamp_valid &&
        now_us - timeout_reference_us >
        (uint32_t)controller->config.vision_timeout_ms * 1000U)
    {
        ball_control_enter_fault(controller, BALL_CONTROL_FAULT_VISION_TIMEOUT, now_us);
    }
}

float ball_control_get_target_velocity_mm_s(const ball_control_t *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }
    return controller->target_velocity_mm_s;
}

float ball_control_get_target_tube_angle_deg(const ball_control_t *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }
    return controller->target_tube_angle_deg;
}
