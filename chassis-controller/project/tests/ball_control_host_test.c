#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "../code/ball_actuator.h"
#include "../code/ball_control.h"

static void test_actuator_mapping_and_rate_limit(void)
{
    ball_actuator_config_t config;
    ball_actuator_t actuator;

    ball_actuator_config_defaults(&config);
    assert(ball_actuator_init(&actuator, &config, 0U));
    assert(ball_actuator_map_angle_to_pulse_us(&actuator, -5.0f) == 1350U);
    assert(ball_actuator_map_angle_to_pulse_us(&actuator, 0.0f) == 1500U);
    assert(ball_actuator_map_angle_to_pulse_us(&actuator, 5.0f) == 1650U);

    ball_actuator_command_angle(&actuator, 5.0f, 5000U);
    assert(ball_actuator_get_pulse_us(&actuator) == 1506U);

    config.pulse_at_min_angle_us = 1650U;
    config.pulse_at_max_angle_us = 1350U;
    config.pulse_rate_limit_us_s = 0.0f;
    assert(ball_actuator_init(&actuator, &config, 0U));
    assert(ball_actuator_map_angle_to_pulse_us(&actuator, -5.0f) == 1650U);
    assert(ball_actuator_map_angle_to_pulse_us(&actuator, 5.0f) == 1350U);
}

static void test_cascade_direction_speed_limit_and_timeout(void)
{
    ball_control_config_t config;
    ball_control_t controller;
    ball_control_measurement_t measurement = {0};

    ball_control_config_defaults(&config);
    config.position_filter_tau_s = 0.0f;
    config.velocity_filter_tau_s = 0.0f;
    config.acceleration_filter_tau_s = 0.0f;
    config.position_kp_per_s = 2.0f;
    config.target_speed_limit_mm_s = 20.0f;
    config.velocity_kp_deg_s_per_mm = 0.05f;
    config.tube_angle_rate_limit_deg_s = 0.0f;
    config.arming_valid_frames = 2U;

    ball_control_init(&controller, &config);
    ball_control_set_target_mm(&controller, 50.0f);
    ball_control_request_enable(&controller, true, 0U);

    measurement.sequence = 1U;
    measurement.capture_timestamp_us = 10000U;
    measurement.position_mm = 0.0f;
    measurement.confidence = 1.0f;
    assert(ball_control_consume_measurement(&controller, &measurement, 10000U));

    measurement.sequence = 2U;
    measurement.capture_timestamp_us = 30000U;
    assert(ball_control_consume_measurement(&controller, &measurement, 30000U));
    assert(controller.state == BALL_CONTROL_TRACKING);
    assert(fabsf(ball_control_get_target_velocity_mm_s(&controller) - 20.0f) < 0.001f);
    assert(ball_control_get_target_tube_angle_deg(&controller) > 0.0f);

    ball_control_set_target_mm(&controller, -50.0f);
    measurement.sequence = 3U;
    measurement.capture_timestamp_us = 50000U;
    assert(ball_control_consume_measurement(&controller, &measurement, 50000U));
    assert(ball_control_get_target_velocity_mm_s(&controller) < 0.0f);
    assert(ball_control_get_target_tube_angle_deg(&controller) < 0.0f);

    ball_control_tick(&controller, 231000U);
    assert(controller.state == BALL_CONTROL_FAULT_LATCHED);
    assert(controller.fault == BALL_CONTROL_FAULT_VISION_TIMEOUT);
    assert(ball_control_get_target_tube_angle_deg(&controller) == 0.0f);
}

static void test_position_integral_limit_and_hard_limit(void)
{
    ball_control_config_t config;
    ball_control_t controller;
    ball_control_measurement_t measurement = {0};
    uint32_t capture_timestamp_us = 10000U;

    ball_control_config_defaults(&config);
    config.position_filter_tau_s = 0.0f;
    config.velocity_filter_tau_s = 0.0f;
    config.position_kp_per_s = 0.0f;
    config.position_ki_per_s2 = 1.0f;
    config.position_integral_zone_mm = 10.0f;
    config.position_integral_limit_mm_s = 0.2f;
    config.target_speed_limit_mm_s = 100.0f;
    config.velocity_kp_deg_s_per_mm = 0.05f;
    config.tube_angle_rate_limit_deg_s = 0.0f;
    config.arming_valid_frames = 2U;

    ball_control_init(&controller, &config);
    ball_control_set_target_mm(&controller, 5.0f);
    ball_control_request_enable(&controller, true, 0U);

    measurement.position_mm = 0.0f;
    measurement.confidence = 1.0f;
    for (uint32_t sequence = 1U; sequence <= 20U; ++sequence)
    {
        measurement.sequence = sequence;
        measurement.capture_timestamp_us = capture_timestamp_us;
        assert(ball_control_consume_measurement(&controller,
                                                &measurement,
                                                capture_timestamp_us));
        capture_timestamp_us += 20000U;
    }

    assert(controller.position_integral_mm_s <= 0.2001f);
    assert(controller.position_integral_mm_s >= 0.1999f);

    measurement.sequence++;
    measurement.capture_timestamp_us = capture_timestamp_us;
    measurement.position_mm = 121.0f;
    assert(!ball_control_consume_measurement(&controller,
                                             &measurement,
                                             capture_timestamp_us));
    assert(controller.fault == BALL_CONTROL_FAULT_POSITION_HARD_LIMIT);
}

int main(void)
{
    test_actuator_mapping_and_rate_limit();
    test_cascade_direction_speed_limit_and_timeout();
    test_position_integral_limit_and_hard_limit();
    puts("ball control host tests passed");
    return 0;
}
