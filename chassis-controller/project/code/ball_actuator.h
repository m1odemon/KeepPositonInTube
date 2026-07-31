#ifndef BALL_ACTUATOR_H
#define BALL_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Tube-angle to servo-pulse calibration.
 *
 * The three pulse calibration points are intentionally independent so that
 * linkage asymmetry and reversed servo direction do not leak into the PID
 * gains.  Replace the piecewise-linear mapping later if the measured mechanism
 * requires a lookup table or the exact linkage model.
 */
typedef struct
{
    float tube_angle_min_deg;
    float tube_angle_max_deg;
    uint16_t pulse_at_min_angle_us;
    uint16_t pulse_neutral_us;
    uint16_t pulse_at_max_angle_us;
    uint16_t pulse_safe_us;
    uint16_t pulse_hard_min_us;
    uint16_t pulse_hard_max_us;
    float pulse_rate_limit_us_s;
} ball_actuator_config_t;

typedef struct
{
    ball_actuator_config_t config;
    float requested_tube_angle_deg;
    uint16_t pulse_us;
    uint32_t last_update_timestamp_us;
    bool timestamp_valid;
} ball_actuator_t;

void ball_actuator_config_defaults(ball_actuator_config_t *config);
bool ball_actuator_init(ball_actuator_t *actuator,
                        const ball_actuator_config_t *config,
                        uint32_t now_us);

uint16_t ball_actuator_map_angle_to_pulse_us(const ball_actuator_t *actuator,
                                              float tube_angle_deg);
void ball_actuator_command_angle(ball_actuator_t *actuator,
                                 float tube_angle_deg,
                                 uint32_t now_us);
void ball_actuator_force_safe(ball_actuator_t *actuator, uint32_t now_us);

float ball_actuator_get_requested_angle_deg(const ball_actuator_t *actuator);
uint16_t ball_actuator_get_pulse_us(const ball_actuator_t *actuator);

#endif
