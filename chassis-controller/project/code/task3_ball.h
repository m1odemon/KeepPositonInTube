#ifndef TASK3_BALL_H
#define TASK3_BALL_H

#include <stdint.h>

#include "ball_control.h"

/*
 * H题 Task3：小车静止，钢球从 O 点移动到 +50 mm，再返回并稳定在
 * -50 mm。当前版本故意不连接任何树莓派/UART 数据源。
 */
typedef enum
{
    TASK3_BALL_READY = 0,
    TASK3_BALL_WAIT_SENSOR,
    TASK3_BALL_TO_PLUS_50,
    TASK3_BALL_TO_MINUS_50,
    TASK3_BALL_DONE_HOLD,
    TASK3_BALL_FAULT
} task3_ball_state_enum;

void task3_ball_init(void);
uint8_t task3_ball_start(void);
void task3_ball_stop(void);
void task3_ball_update_5ms(void);

/*
 * 未来的位置检测适配口。必须从普通任务上下文调用，不可直接从 UART ISR
 * 调用。当前工程没有调用者，因此 Task3 会安全停留在 WAIT_SENSOR。
 */
void task3_ball_submit_measurement(const ball_control_measurement_t *measurement);

task3_ball_state_enum task3_ball_get_state(void);
ball_control_fault_enum task3_ball_get_fault(void);
uint32_t task3_ball_get_elapsed_ms(void);
float task3_ball_get_target_mm(void);
float task3_ball_get_position_mm(void);
uint8_t task3_ball_has_measurement(void);
uint8_t task3_ball_is_finished(void);
uint8_t task3_ball_is_overtime(void);

#endif
