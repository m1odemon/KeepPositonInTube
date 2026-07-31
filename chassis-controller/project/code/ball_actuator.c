#include "ball_actuator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float ball_actuator_clip_float(float value, float low, float high)
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

static uint16_t ball_actuator_clip_pulse(const ball_actuator_t *actuator,
                                         float pulse_us)
{
    float clipped = ball_actuator_clip_float(
        pulse_us,
        (float)actuator->config.pulse_hard_min_us,
        (float)actuator->config.pulse_hard_max_us);
    return (uint16_t)(clipped + 0.5f);
}

void ball_actuator_config_defaults(ball_actuator_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->tube_angle_min_deg = -5.0f;
    config->tube_angle_max_deg = 5.0f;
    config->pulse_at_min_angle_us = 1350U;
    config->pulse_neutral_us = 1500U;
    config->pulse_at_max_angle_us = 1650U;
    config->pulse_safe_us = 1500U;
    config->pulse_hard_min_us = 1100U;
    config->pulse_hard_max_us = 1900U;
    config->pulse_rate_limit_us_s = 1200.0f;
}

bool ball_actuator_init(ball_actuator_t *actuator,
                        const ball_actuator_config_t *config,
                        uint32_t now_us)
{
    ball_actuator_config_t default_config;

    if (actuator == NULL)
    {
        return false;
    }

    if (config == NULL)
    {
        ball_actuator_config_defaults(&default_config);
        config = &default_config;
    }

    memset(actuator, 0, sizeof(*actuator));
    actuator->config = *config;

    if (actuator->config.tube_angle_min_deg >= 0.0f)
    {
        actuator->config.tube_angle_min_deg = -1.0f;
    }
    if (actuator->config.tube_angle_max_deg <= 0.0f)
    {
        actuator->config.tube_angle_max_deg = 1.0f;
    }
    if (actuator->config.pulse_hard_min_us >=
        actuator->config.pulse_hard_max_us)
    {
        actuator->config.pulse_hard_min_us = 1100U;
        actuator->config.pulse_hard_max_us = 1900U;
    }

    actuator->config.pulse_at_min_angle_us =
        ball_actuator_clip_pulse(actuator,
                                 (float)actuator->config.pulse_at_min_angle_us);
    actuator->config.pulse_neutral_us =
        ball_actuator_clip_pulse(actuator,
                                 (float)actuator->config.pulse_neutral_us);
    actuator->config.pulse_at_max_angle_us =
        ball_actuator_clip_pulse(actuator,
                                 (float)actuator->config.pulse_at_max_angle_us);
    actuator->config.pulse_safe_us =
        ball_actuator_clip_pulse(actuator,
                                 (float)actuator->config.pulse_safe_us);

    actuator->requested_tube_angle_deg = 0.0f;
    actuator->pulse_us = actuator->config.pulse_safe_us;
    actuator->last_update_timestamp_us = now_us;
    actuator->timestamp_valid = true;
    return true;
}

uint16_t ball_actuator_map_angle_to_pulse_us(const ball_actuator_t *actuator,
                                              float tube_angle_deg)
{
    if (actuator == NULL || !isfinite(tube_angle_deg))
    {
        return 0U;
    }

    float angle_deg = ball_actuator_clip_float(
        tube_angle_deg,
        actuator->config.tube_angle_min_deg,
        actuator->config.tube_angle_max_deg);
    float pulse_us;

    if (angle_deg >= 0.0f)
    {
        float ratio = angle_deg / actuator->config.tube_angle_max_deg;
        pulse_us = (float)actuator->config.pulse_neutral_us
                 + ratio
                   * ((float)actuator->config.pulse_at_max_angle_us
                      - (float)actuator->config.pulse_neutral_us);
    }
    else
    {
        float ratio = angle_deg / actuator->config.tube_angle_min_deg;
        pulse_us = (float)actuator->config.pulse_neutral_us
                 + ratio
                   * ((float)actuator->config.pulse_at_min_angle_us
                      - (float)actuator->config.pulse_neutral_us);
    }

    return ball_actuator_clip_pulse(actuator, pulse_us);
}

void ball_actuator_command_angle(ball_actuator_t *actuator,
                                 float tube_angle_deg,
                                 uint32_t now_us)
{
    if (actuator == NULL || !isfinite(tube_angle_deg))
    {
        return;
    }

    actuator->requested_tube_angle_deg = ball_actuator_clip_float(
        tube_angle_deg,
        actuator->config.tube_angle_min_deg,
        actuator->config.tube_angle_max_deg);
    uint16_t requested_pulse =
        ball_actuator_map_angle_to_pulse_us(actuator,
                                            actuator->requested_tube_angle_deg);

    if (!actuator->timestamp_valid)
    {
        actuator->pulse_us = requested_pulse;
        actuator->last_update_timestamp_us = now_us;
        actuator->timestamp_valid = true;
        return;
    }

    uint32_t elapsed_us = now_us - actuator->last_update_timestamp_us;
    if (actuator->config.pulse_rate_limit_us_s > 0.0f && elapsed_us > 0U)
    {
        float maximum_step_us = actuator->config.pulse_rate_limit_us_s
                              * ((float)elapsed_us / 1000000.0f);
        float next_pulse_us = ball_actuator_clip_float(
            (float)requested_pulse,
            (float)actuator->pulse_us - maximum_step_us,
            (float)actuator->pulse_us + maximum_step_us);
        actuator->pulse_us = ball_actuator_clip_pulse(actuator, next_pulse_us);
    }
    else
    {
        actuator->pulse_us = requested_pulse;
    }

    actuator->last_update_timestamp_us = now_us;
}

void ball_actuator_force_safe(ball_actuator_t *actuator, uint32_t now_us)
{
    if (actuator == NULL)
    {
        return;
    }

    actuator->requested_tube_angle_deg = 0.0f;
    actuator->pulse_us = actuator->config.pulse_safe_us;
    actuator->last_update_timestamp_us = now_us;
    actuator->timestamp_valid = true;
}

float ball_actuator_get_requested_angle_deg(const ball_actuator_t *actuator)
{
    return (actuator != NULL) ? actuator->requested_tube_angle_deg : 0.0f;
}

uint16_t ball_actuator_get_pulse_us(const ball_actuator_t *actuator)
{
    return (actuator != NULL) ? actuator->pulse_us : 0U;
}
