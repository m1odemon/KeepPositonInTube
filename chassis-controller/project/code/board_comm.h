#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "zf_common_headfile.h"

/* Task10 PC PID-tuning UART defaults. Raspberry Pi task protocols are disabled. */
#ifndef BOARD_COMM_UART
#define BOARD_COMM_UART        UART_2
#endif

#ifndef BOARD_COMM_BAUDRATE
#define BOARD_COMM_BAUDRATE    115200
#endif

#ifndef BOARD_COMM_TX_PIN
#define BOARD_COMM_TX_PIN      UART2_TX_B15
#endif

#ifndef BOARD_COMM_RX_PIN
#define BOARD_COMM_RX_PIN      UART2_RX_B16
#endif

void comm_init(void);

// Task10 PID 调参专用（给 llm-pid-tuner 用）
void send_pid_tuning_status(uint32 timestamp_ms,
                            float setpoint,
                            float input,
                            float pwm,
                            float error,
                            float p,
                            float i,
                            float d);
void pid_tuning_rx_byte(uint8 byte);
void pid_tuning_process_command(void);

#endif // !BOARD_COMM_H
