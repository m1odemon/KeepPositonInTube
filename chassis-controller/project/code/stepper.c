#include "stepper.h"

static uint32 stepper_speed_hz = 0U;

static uint32 stepper_abs_speed_hz(int32 speed_hz)
{
    /* This form also handles the most-negative int32 value safely. */
    if (speed_hz < 0)
    {
        return (uint32)(-(speed_hz + 1)) + 1U;
    }

    return (uint32)speed_hz;
}

static uint32 stepper_limit_speed_hz(uint32 speed_hz)
{
    if (speed_hz < STEPPER_MIN_HZ)
    {
        return STEPPER_MIN_HZ;
    }

    if (speed_hz > STEPPER_MAX_HZ)
    {
        return STEPPER_MAX_HZ;
    }

    return speed_hz;
}

void stepper_init(void)
{
    gpio_init(STEPPER_DIR_PIN, GPO, STEPPER_FORWARD_LEVEL, GPO_PUSH_PULL);

    /* TIMG12 is unused by the chassis, wheel PWM, and servo modules. */
    pwm_init(STEPPER_STEP_PWM, STEPPER_MIN_HZ, 0U);
    stepper_speed_hz = 0U;
}

void stepper_set_speed_hz(int32 speed_hz)
{
    uint32 frequency;

    if (speed_hz == 0)
    {
        stepper_stop();
        return;
    }

    gpio_set_level(STEPPER_DIR_PIN,
                   (speed_hz > 0) ? STEPPER_FORWARD_LEVEL
                                  : !STEPPER_FORWARD_LEVEL);

    /* Meets the DIR-to-STEP setup time of common STEP/DIR drivers. */
    system_delay_us(2U);

    frequency = stepper_limit_speed_hz(stepper_abs_speed_hz(speed_hz));

    if (frequency != stepper_speed_hz)
    {
        /* Suppress pulses while the timer period is being changed. */
        pwm_set_duty(STEPPER_STEP_PWM, 0U);
        pwm_init(STEPPER_STEP_PWM, frequency, PWM_DUTY_MAX / 2U);
        stepper_speed_hz = frequency;
    }
    else
    {
        pwm_set_duty(STEPPER_STEP_PWM, PWM_DUTY_MAX / 2U);
    }
}

void stepper_stop(void)
{
    pwm_set_duty(STEPPER_STEP_PWM, 0U);
    stepper_speed_hz = 0U;
}

uint32 stepper_get_speed_hz(void)
{
    return stepper_speed_hz;
}
