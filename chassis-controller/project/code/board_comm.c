#include "board_comm.h"
#include "adc_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

/*
 * Task10 only tunes the outer yaw PID.  The tracking error is limited to
 * +/-13, so these bounds retain a useful tuning range while preventing a
 * malformed serial command from reversing the loop or immediately driving
 * it into the output limits.
 */
#define PID_TUNING_KP_MIN     (0.0f)
#define PID_TUNING_KP_MAX     (40.0f)
#define PID_TUNING_KI_MIN     (0.0f)
#define PID_TUNING_KI_MAX     (5.0f)
#define PID_TUNING_KD_MIN     (0.0f)
#define PID_TUNING_KD_MAX     (20.0f)

// Task10 PID tuning command cache (ASCII line protocol)
static volatile uint8 pid_cmd_ready = 0;
static char pid_cmd_line[80] = {0};
static uint8 pid_cmd_index = 0;

void send_pid_tuning_status(uint32 timestamp_ms,
                            float setpoint,
                            float input,
                            float pwm,
                            float error,
                            float p,
                            float i,
                            float d)
{
    char tx_line[160];
    int len = snprintf(tx_line,
                       sizeof(tx_line),
                       "%lu,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                       (unsigned long)timestamp_ms,
                       setpoint,
                       input,
                       pwm,
                       error,
                       p,
                       i,
                       d);

    if (len > 0)
    {
        if ((uint32)len >= sizeof(tx_line))
        {
            len = sizeof(tx_line) - 1;
        }
        uart_write_buffer(BOARD_COMM_UART, (const uint8 *)tx_line, (uint32)len);
    }
}

void pid_tuning_rx_byte(uint8 byte)
{
    if (byte == '\r')
    {
        return;
    }

    if (byte == '\n')
    {
        if (pid_cmd_index > 0)
        {
            pid_cmd_line[pid_cmd_index] = '\0';
            pid_cmd_ready = 1;
            pid_cmd_index = 0;
        }
        return;
    }

    if (pid_cmd_ready)
    {
        // Previous command is not processed yet; drop new bytes.
        return;
    }

    if (pid_cmd_index < (sizeof(pid_cmd_line) - 1))
    {
        pid_cmd_line[pid_cmd_index++] = (char)byte;
    }
    else
    {
        // Overflow: discard this line.
        pid_cmd_index = 0;
    }
}

static uint8 pid_tuning_parse_set_command(const char *line,
                                          float *new_p,
                                          float *new_i,
                                          float *new_d)
{
    const char *cursor = line + 4;
    uint8 p_found = 0U;
    uint8 i_found = 0U;
    uint8 d_found = 0U;
    int key_style = -1;

    while (*cursor != '\0')
    {
        const char *token_start;
        const char *token_end;
        const char *value_start;
        char *endptr = NULL;
        float value;
        char field;
        int token_key_style;

        while (isspace((unsigned char)*cursor))
        {
            cursor++;
        }

        if (*cursor == '\0')
        {
            break;
        }

        token_start = cursor;
        while (*cursor != '\0' && !isspace((unsigned char)*cursor))
        {
            cursor++;
        }
        token_end = cursor;

        if ((token_end - token_start) >= 2 &&
            (token_start[0] == 'P' || token_start[0] == 'I' || token_start[0] == 'D') &&
            token_start[1] == ':')
        {
            field = token_start[0];
            value_start = token_start + 2;
            token_key_style = 0;
        }
        else if ((token_end - token_start) >= 3 &&
                 token_start[0] == 'K' &&
                 (token_start[1] == 'P' || token_start[1] == 'I' || token_start[1] == 'D') &&
                 token_start[2] == ':')
        {
            field = token_start[1];
            value_start = token_start + 3;
            token_key_style = 1;
        }
        else
        {
            return 0U;
        }

        if (key_style < 0)
        {
            key_style = token_key_style;
        }
        else if (key_style != token_key_style)
        {
            return 0U;
        }

        value = strtof(value_start, &endptr);
        if (endptr == value_start || endptr != token_end)
        {
            return 0U;
        }

        if (field == 'P')
        {
            if (p_found)
            {
                return 0U;
            }
            *new_p = value;
            p_found = 1U;
        }
        else if (field == 'I')
        {
            if (i_found)
            {
                return 0U;
            }
            *new_i = value;
            i_found = 1U;
        }
        else
        {
            if (d_found)
            {
                return 0U;
            }
            *new_d = value;
            d_found = 1U;
        }
    }

    return (uint8)(p_found && i_found && d_found);
}

static uint8 pid_tuning_value_is_valid(float value, float minimum, float maximum)
{
    return (uint8)(isfinite(value) && value >= minimum && value <= maximum);
}

static void pid_tuning_send_text(const char *text)
{
    uart_write_string(BOARD_COMM_UART, text);
    uart_write_string(BOARD_COMM_UART, "\r\n");
}

void pid_tuning_process_command(void)
{
    if (!pid_cmd_ready)
    {
        return;
    }

    char local_cmd[80] = {0};
    uint32 primask = interrupt_global_disable();
    memcpy(local_cmd, pid_cmd_line, sizeof(local_cmd));
    pid_cmd_ready = 0;
    interrupt_global_enable(primask);

    if (0 == strcmp(local_cmd, "STATUS"))
    {
        char resp[120];

        primask = interrupt_global_disable();
        float kp = PIDK_YA.Kp;
        float ki = PIDK_YA.Ki;
        float kd = PIDK_YA.Kd;
        interrupt_global_enable(primask);

        snprintf(resp, sizeof(resp), "# STATUS: Kp=%.6f Ki=%.6f Kd=%.6f", kp, ki, kd);
        pid_tuning_send_text(resp);
        return;
    }

    if (0 == strncmp(local_cmd, "SET ", 4))
    {
        float new_p = 0.0f;
        float new_i = 0.0f;
        float new_d = 0.0f;

        if (pid_tuning_parse_set_command(local_cmd, &new_p, &new_i, &new_d))
        {
            if (!pid_tuning_value_is_valid(new_p, PID_TUNING_KP_MIN, PID_TUNING_KP_MAX) ||
                !pid_tuning_value_is_valid(new_i, PID_TUNING_KI_MIN, PID_TUNING_KI_MAX) ||
                !pid_tuning_value_is_valid(new_d, PID_TUNING_KD_MIN, PID_TUNING_KD_MAX))
            {
                pid_tuning_send_text("# ERR: PID_VALUE_OUT_OF_RANGE");
                return;
            }

            primask = interrupt_global_disable();
            PIDK_YA.Kp = new_p;
            PIDK_YA.Ki = new_i;
            PIDK_YA.Kd = new_d;
            reset_pid();
            line_error_filtered = 0.0f;
            Duty_dS = 0.0f;
            interrupt_global_enable(primask);

            char ack[120];
            snprintf(ack, sizeof(ack), "# ACK: Kp=%.6f Ki=%.6f Kd=%.6f", new_p, new_i, new_d);
            pid_tuning_send_text(ack);
            return;
        }

        pid_tuning_send_text("# ERR: BAD_SET_FORMAT");
        return;
    }

    pid_tuning_send_text("# ERR: UNKNOWN_CMD");
}

void comm_init(void)
{
    uart_init(BOARD_COMM_UART,
              BOARD_COMM_BAUDRATE,
              BOARD_COMM_TX_PIN,
              BOARD_COMM_RX_PIN);
    uart_set_interrupt_config(BOARD_COMM_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);
}
