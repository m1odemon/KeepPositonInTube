#ifndef HBALL_APP_BALL_BEAM_CONFIG_H
#define HBALL_APP_BALL_BEAM_CONFIG_H

#include "BallBeamController.h"

#include <cstdint>

namespace hball::app_config {

constexpr uint8_t kMotorCanId = 1U;
constexpr uint32_t kControlPeriodMs = 1U;
constexpr uint32_t kTelemetryPeriodMs = 20U;
constexpr uint32_t kMotorDisableRepeatMs = 100U;

/*
 * Stage 1 safety lock:
 *
 * Keep calibration_valid false until the linkage is installed and the seven
 * motor-angle/tube-angle pairs below have been measured. With this flag
 * false, valid UART traffic cannot enable the motor.
 *
 * Tube angle is positive when gravity accelerates the ball toward the motor
 * end of the pipe. Motor angles are relative to motor_zero_angle_rad.
 */
constexpr BallBeamConfig makeBallBeamConfig() {
    BallBeamConfig config{};
    config.calibration_valid = false;
    config.motor_zero_angle_rad = 0.0F;
    config.calibration = {{
        {-0.30F, -0.05235988F},
        {-0.20F, -0.03490659F},
        {-0.10F, -0.01745329F},
        {0.00F, 0.00F},
        {0.10F, 0.01745329F},
        {0.20F, 0.03490659F},
        {0.30F, 0.05235988F},
    }};

    config.minimum_confidence = 0.55F;
    config.valid_frames_to_arm = 5U;
    config.command_timeout_ms = 150U;
    config.motor_feedback_timeout_ms = 50U;
    config.position_min_mm = -110.0F;
    config.position_max_mm = 110.0F;

    config.maximum_ball_speed_mm_s = 140.0F;
    config.maximum_tube_angle_rad = 0.05235988F;
    config.current_limit_a = 0.4F;
    config.safe_level_current_limit_a = 0.2F;
    config.tube_angle_limit_margin_rad = 0.01745329F;

    config.estimator_alpha = 0.65F;
    config.estimator_beta = 0.08F;
    config.position_kp_s = 1.2F;
    config.position_ki_s2 = 0.0F;
    config.velocity_kp_rad_per_mm_s = 0.00020F;
    config.velocity_ki_rad_per_mm = 0.0F;
    config.angle_kp_a_per_rad = 5.0F;
    config.angle_kd_a_per_rad_s = 0.05F;
    config.acceleration_feedforward_gain = 0.0F;
    return config;
}

inline constexpr BallBeamConfig kBallBeamConfig = makeBallBeamConfig();

} // namespace hball::app_config

#endif
