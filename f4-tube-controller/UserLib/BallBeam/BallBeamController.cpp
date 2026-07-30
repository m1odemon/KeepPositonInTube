#include "BallBeamController.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace hball {
namespace {

constexpr float kTwoPi = 2.0F * std::numbers::pi_v<float>;
constexpr float kGravityMps2 = 9.80665F;
constexpr float kMinimumVisionDtS = 0.005F;
constexpr float kMaximumVisionDtS = 0.100F;

float wrapToPi(float angle_rad) {
    angle_rad = std::fmod(angle_rad + std::numbers::pi_v<float>, kTwoPi);
    if (angle_rad < 0.0F) {
        angle_rad += kTwoPi;
    }
    return angle_rad - std::numbers::pi_v<float>;
}

} // namespace

TubeAngleMap::TubeAngleMap(const BallBeamConfig &config) : config_(config) {
    minimum_tube_angle_ = config_.calibration.front().tube_angle_rad;
    maximum_tube_angle_ = minimum_tube_angle_;

    if (!config_.calibration_valid ||
        !std::isfinite(config_.motor_zero_angle_rad)) {
        return;
    }

    int output_direction = 0;
    for (std::size_t i = 0U; i < config_.calibration.size(); ++i) {
        const CalibrationPoint &point = config_.calibration[i];
        if (!std::isfinite(point.motor_relative_angle_rad) ||
            !std::isfinite(point.tube_angle_rad)) {
            return;
        }

        minimum_tube_angle_ =
            std::min(minimum_tube_angle_, point.tube_angle_rad);
        maximum_tube_angle_ =
            std::max(maximum_tube_angle_, point.tube_angle_rad);

        if (i == 0U) {
            continue;
        }

        const CalibrationPoint &previous = config_.calibration[i - 1U];
        if (point.motor_relative_angle_rad <=
            previous.motor_relative_angle_rad) {
            return;
        }

        const float tube_delta =
            point.tube_angle_rad - previous.tube_angle_rad;
        if (tube_delta == 0.0F) {
            return;
        }

        const int direction = tube_delta > 0.0F ? 1 : -1;
        if (output_direction == 0) {
            output_direction = direction;
        } else if (direction != output_direction) {
            return;
        }
    }

    valid_ = true;
}

float TubeAngleMap::motorRelativeAngle(
    const float absolute_motor_angle_rad) const {
    return wrapToPi(absolute_motor_angle_rad - config_.motor_zero_angle_rad);
}

std::size_t TubeAngleMap::segmentFor(
    const float motor_relative_angle_rad) const {
    if (motor_relative_angle_rad <=
        config_.calibration.front().motor_relative_angle_rad) {
        return 0U;
    }

    for (std::size_t i = 0U; i + 1U < config_.calibration.size(); ++i) {
        if (motor_relative_angle_rad <=
            config_.calibration[i + 1U].motor_relative_angle_rad) {
            return i;
        }
    }

    return config_.calibration.size() - 2U;
}

float TubeAngleMap::tubeAngle(const float motor_relative_angle_rad) const {
    if (!valid_) {
        return 0.0F;
    }

    const std::size_t segment = segmentFor(motor_relative_angle_rad);
    const CalibrationPoint &left = config_.calibration[segment];
    const CalibrationPoint &right = config_.calibration[segment + 1U];
    const float ratio =
        (motor_relative_angle_rad - left.motor_relative_angle_rad) /
        (right.motor_relative_angle_rad - left.motor_relative_angle_rad);
    return left.tube_angle_rad +
           ratio * (right.tube_angle_rad - left.tube_angle_rad);
}

float TubeAngleMap::slope(const float motor_relative_angle_rad) const {
    if (!valid_) {
        return 0.0F;
    }

    const std::size_t segment = segmentFor(motor_relative_angle_rad);
    const CalibrationPoint &left = config_.calibration[segment];
    const CalibrationPoint &right = config_.calibration[segment + 1U];
    return (right.tube_angle_rad - left.tube_angle_rad) /
           (right.motor_relative_angle_rad - left.motor_relative_angle_rad);
}

BallBeamController::BallBeamController(const BallBeamConfig &config)
    : config_(config), angle_map_(config) {}

void BallBeamController::reset(const uint32_t now_ms) {
    command_ = {};
    command_.receive_timestamp_ms = now_ms;
    motor_feedback_ = {};
    has_command_ = false;
    has_motor_feedback_ = false;
    estimator_initialized_ = false;
    was_armed_ = false;
    consecutive_valid_frames_ = 0U;
    last_capture_timestamp_us_ = 0U;
    estimated_position_mm_ = 0.0F;
    estimated_velocity_mm_s_ = 0.0F;
    position_integral_ = 0.0F;
    velocity_integral_ = 0.0F;
    target_tube_angle_rad_ = 0.0F;
    actual_tube_angle_rad_ = 0.0F;
    state_ = ControlState::Disabled;
    latched_faults_ = kFaultNone;
    displayed_faults_ = angle_map_.valid() ? kFaultNone
                                           : kFaultCalibrationInvalid;
}

void BallBeamController::acceptCommand(const PiCommand &command) {
    if (has_command_ && !isSequenceNewer(command.sequence, command_.sequence)) {
        return;
    }

    const bool measurement_valid =
        std::isfinite(command.ball_position_mm) &&
        command.ball_position_mm >= config_.position_min_mm &&
        command.ball_position_mm <= config_.position_max_mm &&
        command.confidence >= config_.minimum_confidence;
    const bool target_valid =
        (command.flags & kCommandTargetValid) != 0U &&
        command.target_position_mm >= config_.position_min_mm &&
        command.target_position_mm <= config_.position_max_mm;
    const bool run_requested = (command.flags & kCommandRun) != 0U;
    const bool emergency_stop =
        (command.flags & kCommandEmergencyStop) != 0U;

    command_ = command;
    has_command_ = true;

    if (emergency_stop || !run_requested || !measurement_valid ||
        !target_valid) {
        consecutive_valid_frames_ = 0U;
        if (emergency_stop || !run_requested) {
            was_armed_ = false;
            position_integral_ = 0.0F;
            velocity_integral_ = 0.0F;
            target_tube_angle_rad_ = 0.0F;
        }
    } else if (consecutive_valid_frames_ < UINT8_MAX) {
        ++consecutive_valid_frames_;
    }

    if (measurement_valid) {
        const uint32_t delta_us = estimator_initialized_
                                      ? command.capture_timestamp_us -
                                            last_capture_timestamp_us_
                                      : 0U;
        updateEstimator(command);
        if (target_valid && run_requested && !emergency_stop) {
            float dt_s = static_cast<float>(delta_us) * 1.0e-6F;
            dt_s = clamp(dt_s, kMinimumVisionDtS, kMaximumVisionDtS);
            updateOuterLoops(command, dt_s);
        }
    }
}

void BallBeamController::acceptMotorFeedback(const MotorFeedback &feedback) {
    if (!feedback.valid || !std::isfinite(feedback.angle_rad) ||
        !std::isfinite(feedback.speed_rpm) ||
        !std::isfinite(feedback.current_a)) {
        return;
    }
    motor_feedback_ = feedback;
    has_motor_feedback_ = true;
}

void BallBeamController::setExternalFault(const uint16_t fault) {
    latched_faults_ = static_cast<uint16_t>(latched_faults_ | fault);
}

void BallBeamController::updateEstimator(const PiCommand &command) {
    if (!estimator_initialized_) {
        estimated_position_mm_ = command.ball_position_mm;
        estimated_velocity_mm_s_ = 0.0F;
        last_capture_timestamp_us_ = command.capture_timestamp_us;
        estimator_initialized_ = true;
        return;
    }

    const uint32_t delta_us =
        command.capture_timestamp_us - last_capture_timestamp_us_;
    float dt_s = static_cast<float>(delta_us) * 1.0e-6F;
    dt_s = clamp(dt_s, kMinimumVisionDtS, kMaximumVisionDtS);

    const float predicted_position =
        estimated_position_mm_ + estimated_velocity_mm_s_ * dt_s;
    const float innovation = command.ball_position_mm - predicted_position;
    estimated_position_mm_ =
        predicted_position + config_.estimator_alpha * innovation;
    estimated_velocity_mm_s_ +=
        config_.estimator_beta * innovation / dt_s;
    estimated_velocity_mm_s_ =
        clamp(estimated_velocity_mm_s_, -2.0F * config_.maximum_ball_speed_mm_s,
              2.0F * config_.maximum_ball_speed_mm_s);
    last_capture_timestamp_us_ = command.capture_timestamp_us;
}

void BallBeamController::updateOuterLoops(const PiCommand &command,
                                          const float dt_s) {
    const float position_error =
        command.target_position_mm - estimated_position_mm_;
    position_integral_ += position_error * dt_s;
    position_integral_ = clamp(position_integral_, -500.0F, 500.0F);

    const float target_ball_speed =
        clamp(config_.position_kp_s * position_error +
                  config_.position_ki_s2 * position_integral_,
              -config_.maximum_ball_speed_mm_s,
              config_.maximum_ball_speed_mm_s);
    const float velocity_error =
        target_ball_speed - estimated_velocity_mm_s_;
    velocity_integral_ += velocity_error * dt_s;
    velocity_integral_ = clamp(velocity_integral_, -500.0F, 500.0F);

    float acceleration_feedforward = 0.0F;
    if ((command.flags & kCommandAccelerationValid) != 0U) {
        const float normalized_acceleration =
            clamp(command.chassis_acceleration_mps2 / kGravityMps2,
                  -0.5F, 0.5F);
        acceleration_feedforward =
            config_.acceleration_feedforward_gain *
            std::asin(normalized_acceleration);
    }

    target_tube_angle_rad_ =
        config_.velocity_kp_rad_per_mm_s * velocity_error +
        config_.velocity_ki_rad_per_mm * velocity_integral_ +
        acceleration_feedforward;
    target_tube_angle_rad_ =
        clamp(target_tube_angle_rad_, -config_.maximum_tube_angle_rad,
              config_.maximum_tube_angle_rad);
}

float BallBeamController::computeAngleCurrent(
    const float target_tube_angle_rad,
    const float actual_tube_angle_rad,
    const float actual_tube_rate_rad_s,
    const float current_limit_a) const {
    const float angle_error = target_tube_angle_rad - actual_tube_angle_rad;
    return clamp(config_.angle_kp_a_per_rad * angle_error -
                     config_.angle_kd_a_per_rad_s * actual_tube_rate_rad_s,
                 -current_limit_a, current_limit_a);
}

ControlOutput BallBeamController::step(const uint32_t now_ms) {
    displayed_faults_ = latched_faults_;
    if (!angle_map_.valid()) {
        displayed_faults_ = static_cast<uint16_t>(
            displayed_faults_ | kFaultCalibrationInvalid);
        state_ = ControlState::CalibrationRequired;
        return {false, 0.0F, state_, displayed_faults_};
    }

    if (!has_motor_feedback_) {
        state_ = ControlState::AwaitMotorFeedback;
        return {false, 0.0F, state_, displayed_faults_};
    }

    const uint32_t motor_age =
        ageMs(now_ms, motor_feedback_.receive_timestamp_ms);
    if (motor_age > config_.motor_feedback_timeout_ms) {
        latched_faults_ = static_cast<uint16_t>(
            latched_faults_ | kFaultMotorFeedbackTimeout);
    }

    const float motor_relative_angle =
        angle_map_.motorRelativeAngle(motor_feedback_.angle_rad);
    actual_tube_angle_rad_ = angle_map_.tubeAngle(motor_relative_angle);
    const float minimum_allowed =
        angle_map_.minimumTubeAngle() - config_.tube_angle_limit_margin_rad;
    const float maximum_allowed =
        angle_map_.maximumTubeAngle() + config_.tube_angle_limit_margin_rad;
    if (actual_tube_angle_rad_ < minimum_allowed ||
        actual_tube_angle_rad_ > maximum_allowed) {
        latched_faults_ =
            static_cast<uint16_t>(latched_faults_ | kFaultMotorAngleLimit);
    }

    if (latched_faults_ != kFaultNone) {
        displayed_faults_ = latched_faults_;
        state_ = ControlState::Fault;
        return {false, 0.0F, state_, displayed_faults_};
    }

    const bool emergency_stop =
        has_command_ &&
        (command_.flags & kCommandEmergencyStop) != 0U;
    const bool run_requested =
        has_command_ && (command_.flags & kCommandRun) != 0U;
    if (emergency_stop || !run_requested) {
        state_ = ControlState::Disabled;
        target_tube_angle_rad_ = 0.0F;
        return {false, 0.0F, state_, displayed_faults_};
    }

    const uint32_t command_age =
        ageMs(now_ms, command_.receive_timestamp_ms);
    const bool command_fresh = command_age <= config_.command_timeout_ms;
    const bool command_measurement_valid =
        command_.confidence >= config_.minimum_confidence &&
        command_.ball_position_mm >= config_.position_min_mm &&
        command_.ball_position_mm <= config_.position_max_mm;
    const bool target_valid =
        (command_.flags & kCommandTargetValid) != 0U &&
        command_.target_position_mm >= config_.position_min_mm &&
        command_.target_position_mm <= config_.position_max_mm;

    if (!command_fresh) {
        displayed_faults_ =
            static_cast<uint16_t>(displayed_faults_ | kFaultCommandTimeout);
    }

    if (command_fresh && command_measurement_valid && target_valid &&
        consecutive_valid_frames_ >= config_.valid_frames_to_arm) {
        was_armed_ = true;
        state_ = ControlState::Tracking;
    } else if (was_armed_) {
        state_ = ControlState::SafeLevel;
        target_tube_angle_rad_ = 0.0F;
    } else {
        state_ = ControlState::AwaitVision;
        return {false, 0.0F, state_, displayed_faults_};
    }

    const float motor_rate_rad_s =
        motor_feedback_.speed_rpm * kTwoPi / 60.0F;
    const float tube_rate_rad_s =
        angle_map_.slope(motor_relative_angle) * motor_rate_rad_s;
    const float current_limit =
        state_ == ControlState::SafeLevel
            ? config_.safe_level_current_limit_a
            : config_.current_limit_a;
    const float current_command =
        computeAngleCurrent(target_tube_angle_rad_, actual_tube_angle_rad_,
                            tube_rate_rad_s, current_limit);
    return {true, current_command, state_, displayed_faults_};
}

ControllerSnapshot BallBeamController::snapshot(
    const uint32_t now_ms) const {
    ControllerSnapshot result{};
    result.ball_position_mm = estimated_position_mm_;
    result.ball_velocity_mm_s = estimated_velocity_mm_s_;
    result.target_position_mm =
        has_command_ ? command_.target_position_mm : 0.0F;
    result.target_tube_angle_rad = target_tube_angle_rad_;
    result.actual_tube_angle_rad = actual_tube_angle_rad_;
    result.state = state_;
    result.fault_flags = displayed_faults_;
    result.command_age_ms =
        has_command_ ? ageMs(now_ms, command_.receive_timestamp_ms)
                     : UINT32_MAX;
    result.motor_feedback_age_ms =
        has_motor_feedback_
            ? ageMs(now_ms, motor_feedback_.receive_timestamp_ms)
            : UINT32_MAX;
    return result;
}

float BallBeamController::clamp(const float value, const float minimum,
                                const float maximum) {
    return std::max(minimum, std::min(maximum, value));
}

uint32_t BallBeamController::ageMs(const uint32_t now_ms,
                                   const uint32_t timestamp_ms) {
    return now_ms - timestamp_ms;
}

} // namespace hball
