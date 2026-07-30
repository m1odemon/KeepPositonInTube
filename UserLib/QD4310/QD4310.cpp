#include "QD4310.h"

#include <algorithm>
#include <cmath>
#include <numbers>

bool QD4310::SendCommand(const Command cmd, const uint16_t value) {
    uint8_t tx_buffer[3];
    tx_buffer[0] = static_cast<uint8_t>(cmd);
    tx_buffer[1] = static_cast<uint8_t>(value & 0xFFU);
    tx_buffer[2] = static_cast<uint8_t>((value >> 8U) & 0xFFU);

    if (hcan == nullptr || HAL_CAN_GetState(hcan) == HAL_CAN_STATE_RESET ||
        HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        ++transmit_errors_;
        return false;
    }

    CAN_TxHeaderTypeDef tx_header{};
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.StdId = 0x400U + id;
    tx_header.ExtId = 0U;
    tx_header.TransmitGlobalTime = DISABLE;
    tx_header.DLC = 3U;

    uint32_t tx_mailbox{};
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buffer, &tx_mailbox) !=
        HAL_OK) {
        ++transmit_errors_;
        return false;
    }
    return true;
}

void QD4310::update(const uint8_t feedback[8],
                    const uint32_t receive_timestamp_ms) {
    feedback_sequence_ = feedback_sequence_ + 1U;
    __DMB();

    enabled = (feedback[0] & 0x01U) != 0U;
    const auto current_raw = static_cast<int16_t>(
        static_cast<uint16_t>(feedback[2]) |
        (static_cast<uint16_t>(feedback[3]) << 8U));
    const auto speed_raw = static_cast<int16_t>(
        static_cast<uint16_t>(feedback[4]) |
        (static_cast<uint16_t>(feedback[5]) << 8U));
    const uint16_t angle_raw =
        static_cast<uint16_t>(feedback[6]) |
        static_cast<uint16_t>(static_cast<uint16_t>(feedback[7]) << 8U);

    current = static_cast<float>(current_raw) * 10.0F /
              static_cast<float>(INT16_MAX);
    speed = static_cast<float>(speed_raw) * 1000.0F /
            static_cast<float>(INT16_MAX);
    angle = static_cast<float>(angle_raw) * 2.0F *
            std::numbers::pi_v<float> / static_cast<float>(UINT16_MAX);
    feedback_timestamp_ms_ = receive_timestamp_ms;

    __DMB();
    feedback_sequence_ = feedback_sequence_ + 1U;
}

bool QD4310::getFeedback(Feedback &feedback) const {
    uint32_t before{};
    uint32_t after{};
    do {
        before = feedback_sequence_;
        if ((before & 1U) != 0U) {
            continue;
        }

        __DMB();
        feedback.enabled = enabled;
        feedback.speed_rpm = speed;
        feedback.angle_rad = angle;
        feedback.current_a = current;
        feedback.receive_timestamp_ms = feedback_timestamp_ms_;
        __DMB();

        after = feedback_sequence_;
    } while (before != after || (after & 1U) != 0U);

    feedback.sequence = after / 2U;
    return after != 0U;
}

bool QD4310::setAngle(const float requested_angle) {
    if (!std::isfinite(requested_angle)) {
        return false;
    }

    constexpr float kTwoPi = 2.0F * std::numbers::pi_v<float>;
    const float angle = std::clamp(requested_angle, 0.0F, kTwoPi);
    const auto raw = static_cast<uint16_t>(
        angle / kTwoPi * static_cast<float>(UINT16_MAX));
    return SendCommand(Command::ANGLE, raw);
}

bool QD4310::setStepAngle(const float requested_step_angle) {
    if (!std::isfinite(requested_step_angle)) {
        return false;
    }

    constexpr float kTwoPi = 2.0F * std::numbers::pi_v<float>;
    const float step_angle =
        std::clamp(requested_step_angle, -kTwoPi, kTwoPi);
    const auto raw = static_cast<int16_t>(
        step_angle / kTwoPi * static_cast<float>(INT16_MAX));
    return SendCommand(Command::STEP_ANGLE, static_cast<uint16_t>(raw));
}

bool QD4310::setSpeed(const float requested_speed_rpm) {
    if (!std::isfinite(requested_speed_rpm)) {
        return false;
    }

    const float speed =
        std::clamp(requested_speed_rpm, -1000.0F, 1000.0F);
    const auto raw = static_cast<int16_t>(
        speed / 1000.0F * static_cast<float>(INT16_MAX));
    return SendCommand(Command::SPEED, static_cast<uint16_t>(raw));
}

bool QD4310::setLowSpeed(const float requested_speed_rpm) {
    if (!std::isfinite(requested_speed_rpm)) {
        return false;
    }

    const float speed =
        std::clamp(requested_speed_rpm, -1000.0F, 1000.0F);
    const auto raw = static_cast<int16_t>(
        speed / 1000.0F * static_cast<float>(INT16_MAX));
    return SendCommand(Command::LOW_SPEED, static_cast<uint16_t>(raw));
}

bool QD4310::setCurrent(const float requested_current_a) {
    if (!std::isfinite(requested_current_a)) {
        return false;
    }

    const float current =
        std::clamp(requested_current_a, -10.0F, 10.0F);
    const auto raw = static_cast<int16_t>(
        current / 10.0F * static_cast<float>(INT16_MAX));
    return SendCommand(Command::CURRENT, static_cast<uint16_t>(raw));
}
