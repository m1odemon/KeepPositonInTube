#ifndef HBALL_BALL_BEAM_CONTROLLER_H
#define HBALL_BALL_BEAM_CONTROLLER_H

#include "BallBeamProtocol.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace hball {

constexpr std::size_t kCalibrationPointCount = 7U;

struct CalibrationPoint {
    float motor_relative_angle_rad{};
    float tube_angle_rad{};
};

struct BallBeamConfig {
    bool calibration_valid{};
    float motor_zero_angle_rad{};
    std::array<CalibrationPoint, kCalibrationPointCount> calibration{};

    float minimum_confidence{0.55F};
    uint8_t valid_frames_to_arm{5U};
    uint32_t command_timeout_ms{150U};
    uint32_t motor_feedback_timeout_ms{50U};

    float position_min_mm{-110.0F};
    float position_max_mm{110.0F};
    float maximum_ball_speed_mm_s{140.0F};
    float maximum_tube_angle_rad{0.05235988F};
    float current_limit_a{0.4F};
    float safe_level_current_limit_a{0.2F};
    float tube_angle_limit_margin_rad{0.01745329F};

    float estimator_alpha{0.65F};
    float estimator_beta{0.08F};
    float position_kp_s{1.2F};
    float position_ki_s2{0.0F};
    float velocity_kp_rad_per_mm_s{0.00020F};
    float velocity_ki_rad_per_mm{0.0F};
    float angle_kp_a_per_rad{5.0F};
    float angle_kd_a_per_rad_s{0.05F};
    float acceleration_feedforward_gain{1.0F};
};

struct MotorFeedback {
    bool valid{};
    bool enabled{};
    float angle_rad{};
    float speed_rpm{};
    float current_a{};
    uint32_t receive_timestamp_ms{};
};

struct ControlOutput {
    bool enable_motor{};
    float current_command_a{};
    ControlState state{ControlState::Disabled};
    uint16_t fault_flags{kFaultNone};
};

struct ControllerSnapshot {
    float ball_position_mm{};
    float ball_velocity_mm_s{};
    float target_position_mm{};
    float target_tube_angle_rad{};
    float actual_tube_angle_rad{};
    ControlState state{ControlState::Disabled};
    uint16_t fault_flags{kFaultNone};
    uint32_t command_age_ms{};
    uint32_t motor_feedback_age_ms{};
};

class TubeAngleMap {
public:
    explicit TubeAngleMap(const BallBeamConfig &config);

    [[nodiscard]] bool valid() const { return valid_; }
    [[nodiscard]] float motorRelativeAngle(float absolute_motor_angle_rad) const;
    [[nodiscard]] float tubeAngle(float motor_relative_angle_rad) const;
    [[nodiscard]] float slope(float motor_relative_angle_rad) const;
    [[nodiscard]] float minimumTubeAngle() const { return minimum_tube_angle_; }
    [[nodiscard]] float maximumTubeAngle() const { return maximum_tube_angle_; }

private:
    std::size_t segmentFor(float motor_relative_angle_rad) const;

    const BallBeamConfig &config_;
    bool valid_{};
    float minimum_tube_angle_{};
    float maximum_tube_angle_{};
};

class BallBeamController {
public:
    explicit BallBeamController(const BallBeamConfig &config);

    void reset(uint32_t now_ms);
    void acceptCommand(const PiCommand &command);
    void acceptMotorFeedback(const MotorFeedback &feedback);
    void setExternalFault(uint16_t fault);
    ControlOutput step(uint32_t now_ms);

    [[nodiscard]] ControllerSnapshot snapshot(uint32_t now_ms) const;
    [[nodiscard]] bool calibrationValid() const { return angle_map_.valid(); }

private:
    void updateEstimator(const PiCommand &command);
    void updateOuterLoops(const PiCommand &command, float dt_s);
    float computeAngleCurrent(float target_tube_angle_rad,
                              float actual_tube_angle_rad,
                              float actual_tube_rate_rad_s,
                              float current_limit_a) const;
    static float clamp(float value, float minimum, float maximum);
    static uint32_t ageMs(uint32_t now_ms, uint32_t timestamp_ms);

    const BallBeamConfig &config_;
    TubeAngleMap angle_map_;

    PiCommand command_{};
    MotorFeedback motor_feedback_{};
    bool has_command_{};
    bool has_motor_feedback_{};
    bool estimator_initialized_{};
    bool was_armed_{};
    uint8_t consecutive_valid_frames_{};
    uint32_t last_capture_timestamp_us_{};

    float estimated_position_mm_{};
    float estimated_velocity_mm_s_{};
    float position_integral_{};
    float velocity_integral_{};
    float target_tube_angle_rad_{};
    float actual_tube_angle_rad_{};

    ControlState state_{ControlState::Disabled};
    uint16_t latched_faults_{kFaultNone};
    uint16_t displayed_faults_{kFaultNone};
};

} // namespace hball

#endif
