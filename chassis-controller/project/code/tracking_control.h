#ifndef _TRACKING_CONTROL_H_
#define _TRACKING_CONTROL_H_

#include <stdint.h>
#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "line_sensor.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
} PID_t;

#define DIR_R               (B11)
#define PWM_R               (PWM_TIM_G0_CH0_B10)
#define DIR_L               (A27)
#define PWM_L               (PWM_TIM_G7_CH0_A26)

extern volatile PID_t PIDK_YG;
extern volatile PID_t PIDK_YA;
extern float aim_yaw_g;
extern float Duty_dS;
extern float Duty_dS_applied;

extern float speed_set_duty;
extern float speed_set_diff_L;
extern float speed_set_diff_R;
extern int start_delay;

float curve_tracking_control(void);
void reset_pid(void);
void car_start_init(void);

/*
 * H题停车策略接口。Task2 使用“累计转角后减速 + A点宽线停车”；
 * 其余接口保留用于后续标定和对照。三个策略函数在真正执行停机后
 * 返回1，否则返回0。
 */
void h_stop_strategy_reset(void);
uint8_t h_stop_by_yaw_355(void);
uint8_t h_stop_by_black_line(uint32_t continue_ms);
uint8_t h_stop_by_yaw_slow_black_line(float slow_yaw_deg,
                                      float slow_speed,
                                      uint32_t continue_ms);

/* Task2：H题环线一圈、计时并停回A点。 */
void h_task2_prepare_start(void);
extern volatile uint32_t h_task2_elapsed_ms;
extern volatile uint8_t h_task2_run_finished;
extern volatile int h_task2_stop_delay_ms;

/* A点黑线使用CH3+CH4+CH5归一化值之和；阈值仍需重复实测标定。 */
extern int h_stop_ch345_sum_threshold;
extern volatile int h_stop_ch345_sum;

float PID_Yaw_a(uint8_t cnl, float YAError_input);
void PID_Yaw_gyro(void);
void motor_set_R(float speed);
void motor_set_L(float speed);
void motor_control(void);
void Motor_init(void);
void tracking_control_loop(void);

#endif
