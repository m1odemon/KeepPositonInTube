#ifndef BALL_BALANCE_APP_H
#define BALL_BALANCE_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "ball_actuator.h"
#include "ball_control.h"
#include "ball_vision_link.h"

/*
 * Non-main integration facade for the one-dimensional ball controller.
 *
 * The future application code has only three periodic responsibilities:
 *   1. Forward each UART2 RX byte to ball_balance_app_rx_uart_byte_from_isr().
 *   2. Call ball_balance_app_tick_5ms() from a 5 ms timer task.
 *   3. Call set_target / set_enable from a menu, command, or test routine.
 *
 * This module drives the user-selected B4 servo output (SERVO_CHANNEL_1).
 */
typedef struct
{
    ball_control_t controller;
    ball_actuator_t actuator;
    ball_vision_link_t vision_link;
    bool initialized;
} ball_balance_app_t;

bool ball_balance_app_init(ball_balance_app_t *app,
                           const ball_control_config_t *config,
                           uint32_t now_us);
bool ball_balance_app_init_with_config(
    ball_balance_app_t *app,
    const ball_control_config_t *control_config,
    const ball_actuator_config_t *actuator_config,
    uint32_t now_us);

void ball_balance_app_set_target_mm(ball_balance_app_t *app, float target_position_mm);
void ball_balance_app_set_vehicle_acceleration_mm_s2(ball_balance_app_t *app,
                                                      float acceleration_mm_s2);
void ball_balance_app_set_enable(ball_balance_app_t *app, bool enable, uint32_t now_us);
bool ball_balance_app_reset_fault(ball_balance_app_t *app, uint32_t now_us);
void ball_balance_app_emergency_stop(ball_balance_app_t *app, uint32_t now_us);

void ball_balance_app_rx_uart_byte_from_isr(ball_balance_app_t *app,
                                             uint8_t byte,
                                             uint32_t receive_timestamp_us);

/* Call every 5 ms.  PID is still evaluated only after a fresh valid frame. */
void ball_balance_app_tick_5ms(ball_balance_app_t *app, uint32_t now_us);

const ball_control_t *ball_balance_app_get_controller(const ball_balance_app_t *app);
const ball_actuator_t *ball_balance_app_get_actuator(const ball_balance_app_t *app);
const ball_vision_link_t *ball_balance_app_get_vision_link(const ball_balance_app_t *app);

#endif
