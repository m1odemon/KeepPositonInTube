#ifndef _BATTERY_3S_H_
#define _BATTERY_3S_H_

#include "zf_common_typedef.h"

/* 3S LiPo pack: 12.6 V full, 11.1 V nominal. */
#define BATTERY_ADC_SCALE_V_PER_COUNT      (0.0089388f)
#define BATTERY_3S_NOMINAL_VOLTAGE         (11.1f)
#define BATTERY_3S_LOW_CUTOFF_VOLTAGE      (10.2f)
#define BATTERY_3S_RECOVER_VOLTAGE         (10.8f)
#define BATTERY_3S_LOW_CONFIRM_TICKS       (100U)  /* 100 x 5 ms = 500 ms */
#define BATTERY_3S_SAMPLE_MIN_VOLTAGE       (9.0f)
#define BATTERY_3S_SAMPLE_MAX_VOLTAGE       (13.5f)

/* Conservative first-test limits for 12 V chassis motors. */
#define MOTOR_3S_DEFAULT_SPEED_SCALE       (2.03f)  /* Matches the former 6S default start voltage. */
#define MOTOR_3S_BASE_DUTY_LIMIT           (3000U)
#define MOTOR_3S_PWM_DUTY_LIMIT            (4000U)

/* Call once every 5 ms while the chassis motors are enabled. */
uint8 battery_3s_update_5ms(float pack_voltage, uint8 motors_enabled);

/* A low-voltage trip remains latched until a healthy pack is detected. */
uint8 battery_3s_motor_allowed(void);
void battery_3s_reset_if_recovered(float pack_voltage);

/* Apply 3S voltage compensation and the conservative PWM limit. */
uint32 battery_3s_compensate_motor_duty(uint32 base_duty, float pack_voltage);

#endif
