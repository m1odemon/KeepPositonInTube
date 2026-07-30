#ifndef HBALL_BALL_BEAM_PROTOCOL_H
#define HBALL_BALL_BEAM_PROTOCOL_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace hball {

constexpr uint8_t kProtocolVersion = 1U;
constexpr uint8_t kPiCommandType = 0x10U;
constexpr uint8_t kTelemetryType = 0x90U;
constexpr uint8_t kPiHeader0 = 0xA5U;
constexpr uint8_t kPiHeader1 = 0x5AU;
constexpr uint8_t kTelemetryHeader0 = 0x5AU;
constexpr uint8_t kTelemetryHeader1 = 0xA5U;

constexpr std::size_t kPiCommandFrameSize = 32U;
constexpr std::size_t kTelemetryFrameSize = 54U;

enum PiCommandFlags : uint8_t {
    kCommandRun = 1U << 0,
    kCommandEmergencyStop = 1U << 1,
    kCommandTargetValid = 1U << 2,
    kCommandAccelerationValid = 1U << 3,
};

struct PiCommand {
    uint32_t sequence{};
    uint32_t capture_timestamp_us{};
    float ball_position_mm{};
    float confidence{};
    float target_position_mm{};
    float chassis_acceleration_mps2{};
    uint8_t task_id{};
    uint8_t flags{};
    uint32_t receive_timestamp_ms{};
};

enum class ControlState : uint8_t {
    Disabled = 0U,
    CalibrationRequired = 1U,
    AwaitMotorFeedback = 2U,
    AwaitVision = 3U,
    Tracking = 4U,
    SafeLevel = 5U,
    Fault = 6U,
};

enum FaultFlags : uint16_t {
    kFaultNone = 0U,
    kFaultCalibrationInvalid = 1U << 0,
    kFaultMotorFeedbackTimeout = 1U << 1,
    kFaultCommandTimeout = 1U << 2,
    kFaultMotorAngleLimit = 1U << 3,
    kFaultCanTransmit = 1U << 4,
    kFaultCanPeripheral = 1U << 5,
    kFaultInvalidCommand = 1U << 6,
};

struct Telemetry {
    uint32_t sequence{};
    uint32_t uptime_ms{};
    float ball_position_mm{};
    float ball_velocity_mm_s{};
    float target_position_mm{};
    float target_tube_angle_rad{};
    float actual_tube_angle_rad{};
    float motor_angle_rad{};
    float motor_speed_rpm{};
    float motor_current_a{};
    uint16_t vision_age_ms{};
    uint16_t motor_feedback_age_ms{};
    ControlState state{ControlState::Disabled};
    uint16_t fault_flags{kFaultNone};
    uint8_t flags{};
};

uint16_t crc16CcittFalse(const uint8_t *data, std::size_t length);
bool isSequenceNewer(uint32_t candidate, uint32_t reference);

class PiCommandParser {
public:
    bool push(uint8_t byte, uint32_t receive_timestamp_ms, PiCommand &command);
    void reset();

    [[nodiscard]] uint32_t validFrames() const { return valid_frames_; }
    [[nodiscard]] uint32_t rejectedFrames() const { return rejected_frames_; }

private:
    bool decode(uint32_t receive_timestamp_ms, PiCommand &command) const;
    void resynchronize();

    std::array<uint8_t, kPiCommandFrameSize> buffer_{};
    std::size_t index_{};
    uint32_t valid_frames_{};
    uint32_t rejected_frames_{};
};

std::size_t encodeTelemetry(const Telemetry &telemetry,
                            std::array<uint8_t, kTelemetryFrameSize> &frame);

} // namespace hball

#endif
