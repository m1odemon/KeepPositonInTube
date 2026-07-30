#ifndef __QD4310_H
#define __QD4310_H

#include <cstdint>
#include "can.h"

class QD4310 {
public:
    struct Feedback {
        bool enabled{};
        float speed_rpm{};
        float angle_rad{};
        float current_a{};
        uint32_t receive_timestamp_ms{};
        uint32_t sequence{};
    };

    explicit QD4310(CAN_HandleTypeDef *hcan, const uint8_t id) :
        id(id), hcan(hcan) {}

    bool enable() { return SendCommand(Command::ENABLE, 0x0000U); }
    bool disable() { return SendCommand(Command::DISABLE, 0x0000U); }
    void update(const uint8_t feedback[8], uint32_t receive_timestamp_ms);
    void update(const uint8_t feedback[8]) {
        update(feedback, HAL_GetTick());
    }
    bool getFeedback(Feedback &feedback) const;

    /**
     * @brief 设置电机角度
     * @param _angle 设置的角度,[0,2pi]
     */
    bool setAngle(float _angle);
    /**
     * @brief 设置电机角度
     * @param _step_angle 设置的角度,[-2pi,2pi]
     */
    bool setStepAngle(float _step_angle);
    /**
     * @brief 设置电机转速
     * @param _speed 设置的转速,[-1000,1000]
     */
    bool setSpeed(float _speed);
    /**
     * @brief 设置电机转速
     * @param _speed 设置的转速,[-1000,1000]
     */
    bool setLowSpeed(float _speed);
    /**
     * @brief 设置电机电流
     * @param _current 设置的转速,[-10,10]
     */
    bool setCurrent(float _current);

    [[nodiscard]] uint32_t transmitErrors() const { return transmit_errors_; }

    bool enabled{};
    uint8_t id;      // CAN id
    float speed{};   // in rpm
    float angle{};   // in rad
    float current{}; // in A
private:
    enum class Command :uint8_t {
        NOP = 0x00,
        ENABLE = 0x01,
        DISABLE = 0x02,
        CURRENT = 0x03,
        SPEED = 0x04,
        ANGLE = 0x05,
        LOW_SPEED = 0x06,
        STEP_ANGLE = 0x07
    };

    CAN_HandleTypeDef *hcan{};
    volatile uint32_t feedback_sequence_{};
    uint32_t feedback_timestamp_ms_{};
    uint32_t transmit_errors_{};

    bool SendCommand(Command cmd, uint16_t value);
};

#endif
