#ifndef _APP_STATE_H_
#define _APP_STATE_H_

#define TASK_H_BASE_TRACK             (1)
#define TASK_H_ONE_LAP_STOP          (2)
#define TASK_H_STATIC_BALL_TRANSFER  (3)
#define TASK_H_DRIVE_TO_B             (4)
#define TASK_H_ONE_LAP_CENTER         (5)
#define TASK_H_ONE_LAP_TARGET         (6)
#define TASK_H_OTHER_RESERVED         (7)
#define TASK_H_OVAL_TRACK             (9)
#define TASK_PID_OVAL_TRACK           (10)

#define TASK_H_DISABLED_FIRST  TASK_H_DRIVE_TO_B
#define TASK_H_DISABLED_LAST   TASK_H_OTHER_RESERVED

/* Shared application state. */
extern volatile int switch_page;
extern volatile float battery_voltage;
extern volatile int page;
extern volatile int task_number;
extern volatile int motor_enable;
extern volatile int run_state;

#endif
