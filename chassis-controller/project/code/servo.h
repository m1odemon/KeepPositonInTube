#ifndef _SERVO_H_
#define _SERVO_H_

#include "zf_driver_pwm.h"

/* Servo interfaces on the extension board, matching the official example. */
#define SERVO_PWM_PIN           (PWM_TIM_A1_CH0_B4)  /* Servo interface 1 */
#define SERVO_PWM_PIN_2         (PWM_TIM_A1_CH1_B5)  /* Servo interface 2 */
/* B4/B5 share TIMA1; the chassis 5 ms PIT is assigned to TIMG6. */
#define SERVO_PWM_FREQ          (50U)

/* Standard servo pulse range: 0 degree = 0.5 ms, 180 degree = 2.5 ms. */
#define SERVO_MIN_PULSE_US      (500U)
#define SERVO_MAX_PULSE_US      (2500U)
#define SERVO_MIN_ANGLE         (0.0f)
#define SERVO_MAX_ANGLE         (180.0f)

/* Non-blocking smooth movement parameters. TIMA0 calls servo_update_5ms(). */
#define SERVO_SOFT_STEP_DEG     (1.0f)
#define SERVO_SOFT_DELAY_MS     (20U)
#define SERVO_UPDATE_PERIOD_MS  (5U)
#define SERVO_SOFT_DELAY_TICKS  (SERVO_SOFT_DELAY_MS / SERVO_UPDATE_PERIOD_MS)

typedef enum
{
    SERVO_CHANNEL_1 = 0U,  /* B4 */
    SERVO_CHANNEL_2 = 1U   /* B5 */
} servo_channel_enum;

/* Initialize the default B4 servo output at the midpoint angle. */
uint8 servo_init(void);

/* Initialize only the B4/B5 servo channels defined above. */
uint8 servo_init_channel(servo_channel_enum channel);

/* Queue the default B4 servo target angle; input is clamped to 0..180 degrees. */
uint8 servo_set_angle(float angle);

/* Queue the target angle on a previously initialized B4/B5 servo channel. */
uint8 servo_set_angle_channel(servo_channel_enum channel, float angle);

/* Queue the default B4 servo target pulse width in microseconds. */
uint8 servo_set_pulse_us(uint32 pulse_us);

/* Queue a target pulse width on a previously initialized B4/B5 servo channel. */
uint8 servo_set_pulse_us_channel(servo_channel_enum channel, uint32 pulse_us);

/*
 * Write a calibrated pulse width immediately and cancel any queued soft move.
 * This is the interface intended for frame-driven closed-loop control.
 */
uint8 servo_apply_pulse_us_now(uint32 pulse_us);
uint8 servo_apply_pulse_us_now_channel(servo_channel_enum channel, uint32 pulse_us);

/* Task8 B4-servo test: alternate a small distance around the midpoint. */
uint8 servo_task8_start(void);
void servo_task8_stop(void);
void servo_task8_update_5ms(void);

/* Advance initialized servo channels by one non-blocking 5 ms service tick. */
void servo_update_5ms(void);

#endif
