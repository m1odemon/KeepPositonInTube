#include "BallBeamConfig.h"
#include "BallBeamController.h"
#include "BallBeamProtocol.h"
#include "QD4310.h"
#include "task_public.h"

#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
#include "dma.h"
#include "task.h"
#include "usart.h"

#include <algorithm>
#include <array>
#include <cstdint>

namespace {

using hball::BallBeamController;
using hball::ControlOutput;
using hball::MotorFeedback;
using hball::PiCommand;

constexpr std::size_t kUartRxDmaBufferSize = 128U;
constexpr uint8_t kTelemetryCalibrationValid = 1U << 0;
constexpr uint8_t kTelemetryMotorFeedbackValid = 1U << 1;
constexpr uint8_t kTelemetryMotorReportedEnabled = 1U << 2;

QD4310 ball_motor(&hcan1, hball::app_config::kMotorCanId);
BallBeamController controller(hball::app_config::kBallBeamConfig);
hball::PiCommandParser command_parser;

std::array<uint8_t, kUartRxDmaBufferSize> uart_rx_dma_buffer{};
std::array<uint8_t, hball::kTelemetryFrameSize> telemetry_tx_buffer{};

volatile uint32_t command_guard{};
PiCommand latest_command{};
volatile uint32_t can_error_pending{};
volatile uint32_t uart_error_count{};
uint32_t telemetry_sequence{};

void publishCommandFromIsr(const PiCommand &command) {
    command_guard = command_guard + 1U;
    __DMB();
    latest_command = command;
    __DMB();
    command_guard = command_guard + 1U;
}

bool copyLatestCommand(PiCommand &command, uint32_t &published_guard) {
    uint32_t before{};
    uint32_t after{};
    do {
        before = command_guard;
        if ((before & 1U) != 0U) {
            continue;
        }
        __DMB();
        command = latest_command;
        __DMB();
        after = command_guard;
    } while (before != after || (after & 1U) != 0U);

    if (after == published_guard || after == 0U) {
        return false;
    }
    published_guard = after;
    return true;
}

bool startUartReceive() {
    const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart6, uart_rx_dma_buffer.data(), uart_rx_dma_buffer.size());
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
        return true;
    }
    return false;
}

bool initializeCan() {
    CAN_FilterTypeDef filter{};
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh =
        static_cast<uint16_t>((0x500U + hball::app_config::kMotorCanId) << 5U);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = static_cast<uint16_t>(0x7FFU << 5U);
    filter.FilterMaskIdLow = 0x0006U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK) {
        return false;
    }

    constexpr uint32_t kCanNotifications =
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING |
        CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR;
    return HAL_CAN_ActivateNotification(&hcan1, kCanNotifications) == HAL_OK;
}

uint16_t saturateAge(const uint32_t age_ms) {
    return static_cast<uint16_t>(std::min<uint32_t>(age_ms, UINT16_MAX));
}

void transmitTelemetry(const uint32_t now_ms,
                       const QD4310::Feedback &motor_feedback,
                       const bool motor_feedback_valid) {
    if (huart6.gState != HAL_UART_STATE_READY) {
        return;
    }

    const hball::ControllerSnapshot snapshot = controller.snapshot(now_ms);
    hball::Telemetry telemetry{};
    telemetry.sequence = telemetry_sequence++;
    telemetry.uptime_ms = now_ms;
    telemetry.ball_position_mm = snapshot.ball_position_mm;
    telemetry.ball_velocity_mm_s = snapshot.ball_velocity_mm_s;
    telemetry.target_position_mm = snapshot.target_position_mm;
    telemetry.target_tube_angle_rad = snapshot.target_tube_angle_rad;
    telemetry.actual_tube_angle_rad = snapshot.actual_tube_angle_rad;
    telemetry.motor_angle_rad =
        motor_feedback_valid ? motor_feedback.angle_rad : 0.0F;
    telemetry.motor_speed_rpm =
        motor_feedback_valid ? motor_feedback.speed_rpm : 0.0F;
    telemetry.motor_current_a =
        motor_feedback_valid ? motor_feedback.current_a : 0.0F;
    telemetry.vision_age_ms = saturateAge(snapshot.command_age_ms);
    telemetry.motor_feedback_age_ms =
        saturateAge(snapshot.motor_feedback_age_ms);
    telemetry.state = snapshot.state;
    telemetry.fault_flags = snapshot.fault_flags;
    telemetry.flags = controller.calibrationValid()
                          ? kTelemetryCalibrationValid
                          : 0U;
    if (motor_feedback_valid) {
        telemetry.flags |= kTelemetryMotorFeedbackValid;
    }
    if (motor_feedback_valid && motor_feedback.enabled) {
        telemetry.flags |= kTelemetryMotorReportedEnabled;
    }

    const std::size_t length =
        hball::encodeTelemetry(telemetry, telemetry_tx_buffer);
    (void)HAL_UART_Transmit_DMA(&huart6, telemetry_tx_buffer.data(), length);
}

} // namespace

extern "C" void StartBallBeamTask(void *argument) {
    (void)argument;
    const uint32_t start_ms = HAL_GetTick();
    controller.reset(start_ms);

    if (!initializeCan()) {
        controller.setExternalFault(hball::kFaultCanPeripheral);
    }
    if (!startUartReceive()) {
        uart_error_count = uart_error_count + 1U;
    }

    (void)ball_motor.setCurrent(0.0F);
    (void)ball_motor.disable();

    TickType_t last_wake_tick = xTaskGetTickCount();
    uint32_t last_command_guard{};
    uint32_t last_motor_sequence{};
    uint32_t last_disable_ms = start_ms;
    uint32_t last_telemetry_ms = start_ms;
    uint8_t consecutive_transmit_failures{};
    bool motor_commanded_enabled{};
    QD4310::Feedback qd_feedback{};
    bool qd_feedback_valid{};

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        PiCommand command{};
        if (copyLatestCommand(command, last_command_guard)) {
            controller.acceptCommand(command);
        }

        QD4310::Feedback candidate_feedback{};
        if (ball_motor.getFeedback(candidate_feedback)) {
            qd_feedback = candidate_feedback;
            qd_feedback_valid = true;
            if (candidate_feedback.sequence != last_motor_sequence) {
                last_motor_sequence = candidate_feedback.sequence;
                MotorFeedback feedback{};
                feedback.valid = true;
                feedback.enabled = candidate_feedback.enabled;
                feedback.angle_rad = candidate_feedback.angle_rad;
                feedback.speed_rpm = candidate_feedback.speed_rpm;
                feedback.current_a = candidate_feedback.current_a;
                feedback.receive_timestamp_ms =
                    candidate_feedback.receive_timestamp_ms;
                controller.acceptMotorFeedback(feedback);
            }
        }

        if (can_error_pending != 0U) {
            can_error_pending = 0U;
            controller.setExternalFault(hball::kFaultCanPeripheral);
        }

        const ControlOutput output = controller.step(now_ms);
        bool transmit_ok = true;
        if (output.enable_motor) {
            if (!motor_commanded_enabled) {
                transmit_ok = ball_motor.enable();
                motor_commanded_enabled = transmit_ok;
            }
            transmit_ok = ball_motor.setCurrent(output.current_command_a) &&
                          transmit_ok;
        } else if (motor_commanded_enabled ||
                   now_ms - last_disable_ms >=
                       hball::app_config::kMotorDisableRepeatMs) {
            transmit_ok = ball_motor.setCurrent(0.0F);
            transmit_ok = ball_motor.disable() && transmit_ok;
            motor_commanded_enabled = false;
            last_disable_ms = now_ms;
        }

        if (!output.enable_motor) {
            consecutive_transmit_failures = 0U;
        } else if (transmit_ok) {
            consecutive_transmit_failures = 0U;
        } else if (consecutive_transmit_failures < UINT8_MAX) {
            ++consecutive_transmit_failures;
            if (consecutive_transmit_failures >= 3U) {
                controller.setExternalFault(hball::kFaultCanTransmit);
            }
        }

        if (now_ms - last_telemetry_ms >=
            hball::app_config::kTelemetryPeriodMs) {
            last_telemetry_ms = now_ms;
            transmitTelemetry(now_ms, qd_feedback, qd_feedback_valid);
        }

        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(hball::app_config::kControlPeriodMs));
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan != &hcan1) {
        return;
    }

    CAN_RxHeaderTypeDef header{};
    uint8_t data[8]{};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK &&
        header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA &&
        header.DLC == 8U &&
        header.StdId ==
            0x500U + hball::app_config::kMotorCanId) {
        ball_motor.update(data, HAL_GetTick());
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        constexpr uint32_t kFatalCanErrors =
            HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF |
            HAL_CAN_ERROR_TIMEOUT;
        const uint32_t errors = HAL_CAN_GetError(hcan);
        if ((errors & kFatalCanErrors) != 0U) {
            can_error_pending = errors & kFatalCanErrors;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                const uint16_t size) {
    if (huart != &huart6) {
        return;
    }

    const uint32_t receive_timestamp_ms = HAL_GetTick();
    PiCommand command{};
    for (uint16_t i = 0U;
         i < size && i < uart_rx_dma_buffer.size(); ++i) {
        if (command_parser.push(uart_rx_dma_buffer[i],
                                receive_timestamp_ms, command)) {
            publishCommandFromIsr(command);
        }
    }

    if (!startUartReceive()) {
        uart_error_count = uart_error_count + 1U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart6) {
        uart_error_count = uart_error_count + 1U;
        command_parser.reset();
        (void)startUartReceive();
    }
}
