#include "battery_3s.h"
#include "zf_driver_pwm.h"
#include <math.h>

static uint16 low_voltage_ticks = 0U;
static uint8 low_voltage_latched = 0U;

static uint8 battery_3s_voltage_is_valid(float pack_voltage)
{
    return (uint8)(isfinite(pack_voltage) &&
                   pack_voltage >= BATTERY_3S_SAMPLE_MIN_VOLTAGE &&
                   pack_voltage <= BATTERY_3S_SAMPLE_MAX_VOLTAGE);
}

uint8 battery_3s_update_5ms(float pack_voltage, uint8 motors_enabled)
{
    if (low_voltage_latched) {
        return 0U;
    }

    if (!motors_enabled) {
        low_voltage_ticks = 0U;
        return 0U;
    }

    if (!battery_3s_voltage_is_valid(pack_voltage) ||
        pack_voltage <= BATTERY_3S_LOW_CUTOFF_VOLTAGE) {
        if (low_voltage_ticks < BATTERY_3S_LOW_CONFIRM_TICKS) {
            low_voltage_ticks++;
        }

        if (low_voltage_ticks >= BATTERY_3S_LOW_CONFIRM_TICKS) {
            low_voltage_latched = 1U;
            return 1U;
        }
    }
    else {
        low_voltage_ticks = 0U;
    }

    return 0U;
}

uint8 battery_3s_motor_allowed(void)
{
    return (uint8)(!low_voltage_latched);
}

void battery_3s_reset_if_recovered(float pack_voltage)
{
    if (battery_3s_voltage_is_valid(pack_voltage) &&
        pack_voltage >= BATTERY_3S_RECOVER_VOLTAGE) {
        low_voltage_ticks = 0U;
        low_voltage_latched = 0U;
    }
}

uint32 battery_3s_compensate_motor_duty(uint32 base_duty, float pack_voltage)
{
    float compensated_duty;

    if (!battery_3s_motor_allowed() ||
        !battery_3s_voltage_is_valid(pack_voltage)) {
        return 0U;
    }

    compensated_duty = (float)base_duty
                     * (BATTERY_3S_NOMINAL_VOLTAGE / pack_voltage);

    if (compensated_duty > (float)MOTOR_3S_PWM_DUTY_LIMIT) {
        return MOTOR_3S_PWM_DUTY_LIMIT;
    }

    if (compensated_duty > (float)PWM_DUTY_MAX) {
        return PWM_DUTY_MAX;
    }

    return (uint32)(compensated_duty + 0.5f);
}
