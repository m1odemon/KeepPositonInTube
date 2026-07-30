#include "BallBeamProtocol.h"

#include <cmath>
#include <cstring>

namespace hball {
namespace {

constexpr uint16_t crc16Byte(uint16_t crc, uint8_t value) {
    crc ^= static_cast<uint16_t>(value) << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        crc = (crc & 0x8000U) != 0U
                  ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                  : static_cast<uint16_t>(crc << 1U);
    }
    return crc;
}

constexpr uint16_t crc16CheckVector() {
    constexpr std::array<uint8_t, 9U> data{
        '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint16_t crc = 0xFFFFU;
    for (const uint8_t value : data) {
        crc = crc16Byte(crc, value);
    }
    return crc;
}

static_assert(crc16CheckVector() == 0x29B1U,
              "CRC-16/CCITT-FALSE implementation is incorrect");

uint16_t readU16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t readU32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

float readFloat(const uint8_t *data) {
    const uint32_t raw = readU32(data);
    float value{};
    static_assert(sizeof(value) == sizeof(raw));
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

void writeU16(uint8_t *data, const uint16_t value) {
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void writeU32(uint8_t *data, const uint32_t value) {
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

void writeFloat(uint8_t *data, const float value) {
    uint32_t raw{};
    static_assert(sizeof(value) == sizeof(raw));
    std::memcpy(&raw, &value, sizeof(raw));
    writeU32(data, raw);
}

bool commandValuesAreValid(const PiCommand &command) {
    constexpr float kPositionLimitMm = 200.0F;
    constexpr float kAccelerationLimitMps2 = 20.0F;

    return std::isfinite(command.ball_position_mm) &&
           std::isfinite(command.confidence) &&
           std::isfinite(command.target_position_mm) &&
           std::isfinite(command.chassis_acceleration_mps2) &&
           command.ball_position_mm >= -kPositionLimitMm &&
           command.ball_position_mm <= kPositionLimitMm &&
           command.target_position_mm >= -kPositionLimitMm &&
           command.target_position_mm <= kPositionLimitMm &&
           command.confidence >= 0.0F && command.confidence <= 1.0F &&
           command.chassis_acceleration_mps2 >= -kAccelerationLimitMps2 &&
           command.chassis_acceleration_mps2 <= kAccelerationLimitMps2;
}

} // namespace

uint16_t crc16CcittFalse(const uint8_t *data, const std::size_t length) {
    uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0U; i < length; ++i) {
        crc = crc16Byte(crc, data[i]);
    }
    return crc;
}

bool isSequenceNewer(const uint32_t candidate, const uint32_t reference) {
    return static_cast<int32_t>(candidate - reference) > 0;
}

void PiCommandParser::reset() {
    index_ = 0U;
}

bool PiCommandParser::push(const uint8_t byte,
                           const uint32_t receive_timestamp_ms,
                           PiCommand &command) {
    if (index_ == 0U) {
        if (byte == kPiHeader0) {
            buffer_[index_++] = byte;
        }
        return false;
    }

    if (index_ == 1U) {
        if (byte == kPiHeader1) {
            buffer_[index_++] = byte;
        } else if (byte == kPiHeader0) {
            buffer_[0] = byte;
        } else {
            index_ = 0U;
        }
        return false;
    }

    buffer_[index_++] = byte;
    if (index_ < buffer_.size()) {
        return false;
    }

    if (decode(receive_timestamp_ms, command)) {
        ++valid_frames_;
        index_ = 0U;
        return true;
    }

    ++rejected_frames_;
    resynchronize();
    return false;
}

bool PiCommandParser::decode(const uint32_t receive_timestamp_ms,
                             PiCommand &command) const {
    if (buffer_[0] != kPiHeader0 || buffer_[1] != kPiHeader1 ||
        buffer_[2] != kProtocolVersion || buffer_[3] != kPiCommandType) {
        return false;
    }

    constexpr std::size_t kCrcDataOffset = 2U;
    constexpr std::size_t kCrcDataLength = 28U;
    const uint16_t received_crc = readU16(buffer_.data() + 30U);
    if (crc16CcittFalse(buffer_.data() + kCrcDataOffset, kCrcDataLength) !=
        received_crc) {
        return false;
    }

    PiCommand candidate{};
    candidate.sequence = readU32(buffer_.data() + 4U);
    candidate.capture_timestamp_us = readU32(buffer_.data() + 8U);
    candidate.ball_position_mm = readFloat(buffer_.data() + 12U);
    candidate.confidence = readFloat(buffer_.data() + 16U);
    candidate.target_position_mm = readFloat(buffer_.data() + 20U);
    candidate.chassis_acceleration_mps2 =
        readFloat(buffer_.data() + 24U);
    candidate.task_id = buffer_[28U];
    candidate.flags = buffer_[29U];
    candidate.receive_timestamp_ms = receive_timestamp_ms;

    if (!commandValuesAreValid(candidate)) {
        return false;
    }

    command = candidate;
    return true;
}

void PiCommandParser::resynchronize() {
    for (std::size_t start = 1U; start + 1U < buffer_.size(); ++start) {
        if (buffer_[start] == kPiHeader0 &&
            buffer_[start + 1U] == kPiHeader1) {
            const std::size_t remaining = buffer_.size() - start;
            std::memmove(buffer_.data(), buffer_.data() + start, remaining);
            index_ = remaining;
            return;
        }
    }

    index_ = buffer_.back() == kPiHeader0 ? 1U : 0U;
    if (index_ == 1U) {
        buffer_[0] = kPiHeader0;
    }
}

std::size_t encodeTelemetry(
    const Telemetry &telemetry,
    std::array<uint8_t, kTelemetryFrameSize> &frame) {
    frame.fill(0U);
    frame[0] = kTelemetryHeader0;
    frame[1] = kTelemetryHeader1;
    frame[2] = kProtocolVersion;
    frame[3] = kTelemetryType;
    writeU32(frame.data() + 4U, telemetry.sequence);
    writeU32(frame.data() + 8U, telemetry.uptime_ms);
    writeFloat(frame.data() + 12U, telemetry.ball_position_mm);
    writeFloat(frame.data() + 16U, telemetry.ball_velocity_mm_s);
    writeFloat(frame.data() + 20U, telemetry.target_position_mm);
    writeFloat(frame.data() + 24U, telemetry.target_tube_angle_rad);
    writeFloat(frame.data() + 28U, telemetry.actual_tube_angle_rad);
    writeFloat(frame.data() + 32U, telemetry.motor_angle_rad);
    writeFloat(frame.data() + 36U, telemetry.motor_speed_rpm);
    writeFloat(frame.data() + 40U, telemetry.motor_current_a);
    writeU16(frame.data() + 44U, telemetry.vision_age_ms);
    writeU16(frame.data() + 46U, telemetry.motor_feedback_age_ms);
    frame[48U] = static_cast<uint8_t>(telemetry.state);
    writeU16(frame.data() + 49U, telemetry.fault_flags);
    frame[51U] = telemetry.flags;

    constexpr std::size_t kCrcDataOffset = 2U;
    constexpr std::size_t kCrcDataLength = 50U;
    writeU16(frame.data() + 52U,
             crc16CcittFalse(frame.data() + kCrcDataOffset, kCrcDataLength));
    return frame.size();
}

} // namespace hball
