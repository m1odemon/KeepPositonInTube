#ifndef STEPPER_H_
#define STEPPER_H_

#include "zf_common_headfile.h"

/*
 * STEP/DIR stepper-driver interface on Motor Driver Signal Port 1.
 *
 *   B13 -> STEP / PUL
 *   B12 -> DIR
 *   GND -> Driver signal ground
 *
 * B8 and B9 on the same connector are reserved by the line-sensor mux.
 * The driver's EN input is intentionally not assigned to an MCU pin. Tie
 * EN to its active level externally, or add a separate GPIO only if required.
 */
#define STEPPER_STEP_PWM         PWM_TIM_G12_CH0_B13
#define STEPPER_DIR_PIN          B12

/* Change this level if the installed motor's positive direction is reversed. */
#define STEPPER_FORWARD_LEVEL    GPIO_HIGH

/* Safe initial range. Tune STEPPER_MAX_HZ after validating the mechanics. */
#define STEPPER_MIN_HZ           (10U)
#define STEPPER_MAX_HZ           (20000U)

/*
 * Initialise B12 and TIMG12/B13. This leaves STEP low, so the motor remains
 * still until stepper_set_speed_hz() is called.
 */
void stepper_init(void);

/*
 * Set continuous speed in STEP pulses per second.
 *   speed_hz > 0: forward
 *   speed_hz < 0: reverse
 *   speed_hz = 0: stop, with STEP held low
 *
 * Call from the main loop or a task state machine, never from a high-frequency
 * interrupt. The driver counts one step at each rising edge of STEP.
 */
void stepper_set_speed_hz(int32 speed_hz);

/* Stop pulse output without changing the selected direction. */
void stepper_stop(void);

/* Returns the currently configured pulse frequency, or 0 when stopped. */
uint32 stepper_get_speed_hz(void);

#endif
