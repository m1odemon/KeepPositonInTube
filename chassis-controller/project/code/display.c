#include "display.h"


// ---------------------------------------------------------
// 1. 参数定义
// ---------------------------------------------------------
uint8_t param_edit_mode = 0;        
uint8_t selected_param = 0;         
uint8_t param_count = 2;
float speed_scale = MOTOR_3S_DEFAULT_SPEED_SCALE;

param_item_t param_list[] = {
    {"Speed",     &speed_scale, 0.1f, 0.1f, 3.0f, 0},
    {"StopDly",   (void*)&h_task2_stop_delay_ms, 5, 0, 500, 1},
};

// ---------------------------------------------------------
// H题行驶时间函数（当前未接入任何主流程）
// ---------------------------------------------------------
#define H_DRIVE_TIMER_TICK_MS  (5U)

static volatile uint32_t h_drive_elapsed_ms = 0U;
static volatile uint8_t h_drive_timer_state = H_DRIVE_TIMER_READY;

void h_drive_timer_reset(void)
{
    /* 先停止计时，再清零，避免以后接入中断时在清零过程中被继续累加。 */
    h_drive_timer_state = H_DRIVE_TIMER_READY;
    h_drive_elapsed_ms = 0U;
}

void h_drive_timer_start(void)
{
    /* 赛题要求按键启动时从0开始计时，因此start同时完成清零。 */
    h_drive_elapsed_ms = 0U;
    h_drive_timer_state = H_DRIVE_TIMER_RUNNING;
}

void h_drive_timer_update_5ms(void)
{
    if (h_drive_timer_state != H_DRIVE_TIMER_RUNNING) {
        return;
    }

    if (h_drive_elapsed_ms <= UINT32_MAX - H_DRIVE_TIMER_TICK_MS) {
        h_drive_elapsed_ms += H_DRIVE_TIMER_TICK_MS;
    } else {
        h_drive_elapsed_ms = UINT32_MAX;
    }
}

void h_drive_timer_stop(void)
{
    if (h_drive_timer_state == H_DRIVE_TIMER_RUNNING) {
        h_drive_timer_state = H_DRIVE_TIMER_STOPPED;
    }
}

uint32_t h_drive_timer_get_ms(void)
{
    return h_drive_elapsed_ms;
}

uint8_t h_drive_timer_get_state(void)
{
    return h_drive_timer_state;
}

/*
 * 在OLED指定行显示固定宽度时间：
 *     Time:012.345s
 *
 * 不使用浮点格式化，确保毫秒部分始终补足3位。显示上限为999.999 s，
 * 超过后只让显示值饱和，内部uint32毫秒计时仍保持真实值。
 */
void h_drive_timer_show(uint8_t row)
{
    char time_text[17] = "Time:000.000s   ";
    uint32_t elapsed_ms;
    uint32_t seconds;
    uint32_t milliseconds;

    if (row >= 8U) {
        return;
    }

    elapsed_ms = h_drive_timer_get_ms();
    if (elapsed_ms > 999999U) {
        elapsed_ms = 999999U;
    }

    seconds = elapsed_ms / 1000U;
    milliseconds = elapsed_ms % 1000U;

    time_text[5] = (char)('0' + (seconds / 100U) % 10U);
    time_text[6] = (char)('0' + (seconds / 10U) % 10U);
    time_text[7] = (char)('0' + seconds % 10U);
    time_text[9] = (char)('0' + (milliseconds / 100U) % 10U);
    time_text[10] = (char)('0' + (milliseconds / 10U) % 10U);
    time_text[11] = (char)('0' + milliseconds % 10U);

    oled_show_string(0, row, time_text);
}

void h_drive_timer_show_state(uint8_t row)
{
    if (row >= 8U) {
        return;
    }

    if (h_drive_timer_state == H_DRIVE_TIMER_RUNNING) {
        oled_show_string(0, row, "Timer:RUN       ");
    } else if (h_drive_timer_state == H_DRIVE_TIMER_STOPPED) {
        oled_show_string(0, row, "Timer:STOPPED   ");
    } else {
        oled_show_string(0, row, "Timer:READY     ");
    }
}

// 保留兼容接口：main.c 当前仍会调用该函数。
int assign_value(void) { return 0; }

#include "servo.h"
#include "app_state.h"
#include "task3_ball.h"

/* OLED has seven menu rows. */
#define TASK_MENU_MIN       (1)
#define TASK_MENU_MAX       (10)
#define TASK_MENU_ROWS      (7)

// ---------------------------------------------------------
// 2. UI 状态机与按键控制
// ---------------------------------------------------------
void menu_control(void) {
    /* KEY4 is a global stop while any task is running, independent of page. */
    if (run_state == 1 && key_get_state(KEY_4) == KEY_SHORT_PRESS) {
        if (task_number == TASK_H_STATIC_BALL_TRANSFER) {
            task3_ball_stop();
        } else if (task_number == 8) {
            servo_task8_stop();
        }
        run_state = 0;
        motor_enable = 0;
        switch_page = 1;
        beep_set_time(50);
        key_clear_state(KEY_4);
        return;
    }

    // ==========================================
    // 模式一：【参数编辑模式】
    // ==========================================
    if(param_edit_mode) {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS) { selected_param = (selected_param > 0) ? selected_param-1 : param_count-1; beep_set_time(50); key_clear_state(KEY_1);}
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS) { selected_param = (selected_param < param_count-1) ? selected_param+1 : 0; beep_set_time(50); key_clear_state(KEY_2);}

        if(key_get_state(KEY_3) == KEY_SHORT_PRESS) 
        {
    // 参数加 (带上限保护，触顶则强制设为max_val)

            if(param_list[selected_param].is_int) 
            {
        
                int next_val = *(int*)param_list[selected_param].value + (int)param_list[selected_param].step;
        
                // 修正：匹配结构体的 max_val 字段
        
                *(int*)param_list[selected_param].value = (next_val <= (int)param_list[selected_param].max_val) ? next_val : (int)param_list[selected_param].max_val;
    
            } 
            else
            {
        
                float next_val = *(float*)param_list[selected_param].value + param_list[selected_param].step;
        
                // 修正：匹配结构体的 max_val 字段
        
                *(float*)param_list[selected_param].value = (next_val <= param_list[selected_param].max_val) ? next_val : param_list[selected_param].max_val;
            }

    
           beep_set_time(50); 
           key_clear_state(KEY_3);
        }
 
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS) 
        {
    // 参数减 (带下限保护，触底则强制设为min_val)
            if(param_list[selected_param].is_int) 
            {
        
                int next_val = *(int*)param_list[selected_param].value - (int)param_list[selected_param].step;
        
                // 修正：匹配结构体的 min_val 字段
                *(int*)param_list[selected_param].value = (next_val >= (int)param_list[selected_param].min_val) ? next_val : (int)param_list[selected_param].min_val;
            } 
            else 
            {
            
                float next_val = *(float*)param_list[selected_param].value - param_list[selected_param].step;
                // 修正：匹配结构体的 min_val 字段
                *(float*)param_list[selected_param].value = (next_val >= param_list[selected_param].min_val) ? next_val : param_list[selected_param].min_val;
            }
    
            beep_set_time(50); 
    
            key_clear_state(KEY_4);

        }
        if(key_get_state(KEY_1) == KEY_LONG_PRESS) { param_edit_mode = 0; switch_page = 1; beep_set_time(200); key_clear_state(KEY_1);}
    } else {
        // ==========================================
        // 模式二：【正常浏览模式】(主菜单)
        // ==========================================
        if(key_get_state(KEY_3) == KEY_LONG_PRESS) { param_edit_mode = 1; switch_page = 1; beep_set_time(200); key_clear_state(KEY_3);} 
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS) { page = (page + 1) % 2; switch_page = 1; beep_set_time(50); key_clear_state(KEY_3);} 

        if(page == 0) {
            // 选择 Task1~Task10
            if(run_state == 0) {
                if(key_get_state(KEY_1) == KEY_SHORT_PRESS) { task_number = (task_number > TASK_MENU_MIN) ? task_number-1 : TASK_MENU_MAX; beep_set_time(50); key_clear_state(KEY_1);}
                if(key_get_state(KEY_2) == KEY_SHORT_PRESS) { task_number = (task_number < TASK_MENU_MAX) ? task_number+1 : TASK_MENU_MIN; beep_set_time(50); key_clear_state(KEY_2);}
            }
            
            // KEY_4：发车 / 紧急停止
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS) {
                if(run_state == 1) {
                    // 正在运行，紧急停止
                    run_state = 0; 
                    motor_enable = 0;
                    switch_page = 1;
                    beep_set_time(50);
                } else {
                    // 准备发车逻辑判断
                    if(task_number == TASK_H_ONE_LAP_STOP) {
                        // H题Task2：环线一圈，计时并在A点宽线自动停车。
                        car_start_init();
                        h_task2_prepare_start();
                        run_state = 1;
                        motor_enable = 1;
                        switch_page = 1;
                        beep_set_time(300);
                    } else if(task_number == TASK_H_STATIC_BALL_TRANSFER) {
                        // H题Task3：底盘保持静止，等待未来的位置检测数据。
                        motor_enable = 0;
                        if (task3_ball_start()) {
                            run_state = 1;
                            switch_page = 1;
                            beep_set_time(300);
                        } else {
                            run_state = 0;
                            switch_page = 1;
                            beep_set_time(50);
                        }
                    } else if (task_number == 8) {
                        // Task8 is a B4-servo-only test.  Wheel motors remain off.
                        if (servo_task8_start()) {
                            run_state = 1;
                            motor_enable = 0;
                            beep_set_time(300);
                        } else {
                            beep_set_time(50);
                        }
                    } else if (task_number == TASK_H_BASE_TRACK ||
                               task_number == TASK_H_OVAL_TRACK ||
                               task_number == TASK_PID_OVAL_TRACK) {
                        // 环形轨道持续循迹测试：只由KEY4或安全保护停止。
                        car_start_init();
                        run_state = 1;
                        motor_enable = 1;
                        beep_set_time(300);
                    } else if (task_number >= TASK_H_DISABLED_FIRST &&
                               task_number <= TASK_H_DISABLED_LAST) {
                        // Task4~7旧工程逻辑已删除；新H题任务接入前禁止启动。
                        motor_enable = 0;
                        run_state = 0;
                        switch_page = 1;
                        beep_set_time(50);
                    } else {
                        // Unsupported task number.
                        beep_set_time(50);
                    }
                }
                key_clear_state(KEY_4);
            }
        }
        else if(page == 1) {
            // 仅在停车状态允许切换 ADC 标定模式，避免运行中误切换导致循迹失效
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS) {
                if(run_state == 0) {
                    adc_mode = (adc_mode + 1) % 3;
                }
                beep_set_time(50);
                key_clear_state(KEY_1);
            }
        }
    }
}

// ---------------------------------------------------------
// 3. 屏幕刷新函数 (主循环调用)
// ---------------------------------------------------------
void show_ui(void) {
    if(switch_page) { oled_clear(); switch_page = 0; }

    if(param_edit_mode) {
        // --- 调参页面 ---
        oled_show_string(0, 0, "- EDIT PARAM -");
        for(int i = 0; i < param_count; i++) {
            uint8_t line = i * 2 + 2;
            if(selected_param == i) oled_show_string(0, line, ">");
            else oled_show_string(0, line, " ");
            
            oled_show_string(8, line, param_list[i].name);
            if(param_list[i].is_int) oled_show_int(16, line+1, *(int*)param_list[i].value, 4);
            else oled_show_float(16, line+1, *(float*)param_list[i].value, 2, 2);
        }
    } else {
        if (page == 0 &&
            task_number == TASK_H_STATIC_BALL_TRANSFER &&
            task3_ball_get_state() != TASK3_BALL_READY) {
            task3_ball_state_enum task3_state = task3_ball_get_state();
            uint32_t elapsed_ms = task3_ball_get_elapsed_ms();

            if (task3_state == TASK3_BALL_DONE_HOLD) {
                oled_show_string(0, 0, "BALL3 [DONE]    ");
            } else if (task3_state == TASK3_BALL_FAULT) {
                oled_show_string(0, 0, "BALL3 [FAULT]   ");
            } else if (task3_state == TASK3_BALL_WAIT_SENSOR) {
                oled_show_string(0, 0, "BALL3 [WAIT]    ");
            } else {
                oled_show_string(0, 0, "BALL3 [RUN]     ");
            }

            oled_show_string(0, 1, "Time:");
            oled_show_float(40, 1, (double)elapsed_ms / 1000.0, 2, 3);
            oled_show_string(104, 1, "s");

            if (task3_state == TASK3_BALL_WAIT_SENSOR) {
                oled_show_string(0, 2, "P:WAIT SENSOR   ");
            } else if (task3_state == TASK3_BALL_TO_PLUS_50) {
                oled_show_string(0, 2, "P:TO +5cm       ");
            } else if (task3_state == TASK3_BALL_TO_MINUS_50) {
                oled_show_string(0, 2, "P:TO -5cm       ");
            } else if (task3_state == TASK3_BALL_DONE_HOLD) {
                oled_show_string(0, 2, "P:HOLD -5cm     ");
            } else {
                oled_show_string(0, 2, "P:SAFE          ");
            }

            oled_show_string(0, 3, "Pos:");
            if (task3_ball_has_measurement()) {
                oled_show_float(32, 3, task3_ball_get_position_mm(), 3, 1);
                oled_show_string(88, 3, "mm");
            } else {
                oled_show_string(32, 3, "--.- mm        ");
            }

            oled_show_string(0, 4, "Target:");
            oled_show_float(56, 4, task3_ball_get_target_mm(), 3, 1);
            oled_show_string(112, 4, "mm");
            oled_show_string(0, 5, task3_ball_is_overtime()
                                   ? "Limit:[OVER 5s] "
                                   : "Limit:5.000s    ");
            oled_show_string(0, 6, "Pi:DISCONNECTED ");
            oled_show_string(0, 7, "K4:SAFE STOP    ");
        }
        else if (page == 0 && task_number == TASK_H_ONE_LAP_STOP &&
            (run_state != 0 || h_task2_elapsed_ms > 0U)) {
            uint32_t elapsed_ms = h_task2_elapsed_ms;

            if (h_task2_run_finished) {
                oled_show_string(0, 0, "H LAP [DONE]    ");
            } else if (run_state) {
                oled_show_string(0, 0, "H LAP [RUN]     ");
            } else {
                oled_show_string(0, 0, "H LAP [STOP]    ");
            }

            oled_show_string(0, 1, "Time:");
            oled_show_float(40, 1, (double)elapsed_ms / 1000.0, 3, 3);
            oled_show_string(112, 1, "s");
            oled_show_string(0, 2, "Yaw:");
            oled_show_float(32, 2, Yaw_TotalAngle, 4, 1);
            oled_show_string(0, 3, "CH345:");
            oled_show_int(48, 3, h_stop_ch345_sum, 3);
            oled_show_string(80, 3, "/300");
            oled_show_string(0, 4, "Thr:");
            oled_show_int(32, 4, h_stop_ch345_sum_threshold, 3);
            oled_show_string(0, 5, "StopDly:");
            oled_show_int(64, 5, h_task2_stop_delay_ms, 3);
            oled_show_string(96, 5, "ms");
            oled_show_string(0, 6, run_state ? "K4:STOP        "
                                              : "K4:RESTART     ");
            oled_show_string(0, 7, "K3:DEBUG       ");
        }
        else if(page == 0) {
            // --- 任务菜单 ---
            // 标题行包含运行状态
            if(run_state) oled_show_string(0, 0, "--TASKS [RUN]--");
            else oled_show_string(0, 0, "--TASKS [STP]--");
            
            // OLED has seven task rows.  Keep the selected task visible;
            // for example Task8 displays Task2..Task8.
            int first_task = (task_number > TASK_MENU_ROWS)
                           ? task_number - (TASK_MENU_ROWS - 1)
                           : TASK_MENU_MIN;
            for (int task_id = first_task;
                 task_id < first_task + TASK_MENU_ROWS;
                 task_id++) {
                uint8_t line = (uint8_t)(task_id - first_task + 1);
                if (task_number == task_id) oled_show_string(0, line, ">");
                else oled_show_string(0, line, " ");

                switch(task_id) {
                    case 1: oled_show_string(8, line, "1:H Track"); break;
                    case 2: oled_show_string(8, line, "2:H Lap Stop"); break;
                    case 3: oled_show_string(8, line, "3:Ball +/-5cm"); break;
                    case 4: oled_show_string(8, line, "4:H A-B [TODO]"); break;
                    case 5: oled_show_string(8, line, "5:H LapO[TODO]"); break;
                    case 6: oled_show_string(8, line, "6:H LapT[TODO]"); break;
                    case 7: oled_show_string(8, line, "7:H Other TODO"); break;
                    case 8: oled_show_string(8, line, "8:Servo Test"); break;
                    case 9: oled_show_string(8, line, "9:H Oval Run"); break;
                    case 10: oled_show_string(8, line, "10:PID UART"); break;
                }
            }
        } 
        else if(page == 1) {
            // --- ADC 和 陀螺仪 调试页 ---
            oled_show_string(0, 0, "Yaw:"); oled_show_float(32, 0, Yaw_TotalAngle, 4, 1);
            oled_show_string(0, 1, "Err:"); oled_show_float(32, 1, line_error_filtered, 4, 1);
            oled_show_string(0, 2, "Bat:"); oled_show_float(32, 2, battery_voltage, 2, 2);
            oled_show_string(80, 2, "Md:"); oled_show_int(104, 2, adc_mode, 1);
            oled_show_string(0, 3, "Yg:");  oled_show_float(32, 3, Yaw_g, 4, 1);
            
            oled_show_string(0, 4, "ADC (L -> R):");
            oled_show_int(0,  5, adc_calibrated_value[0], 3);
            oled_show_int(32, 5, adc_calibrated_value[1], 3);
            oled_show_int(64, 5, adc_calibrated_value[6], 3);
            oled_show_int(96, 5, adc_calibrated_value[7], 3);
            oled_show_string(0, 6, "Cmd:"); oled_show_float(32, 6, aim_yaw_g, 4, 1);
            oled_show_string(0, 7, "Dif:"); oled_show_float(32, 7, Duty_dS_applied, 4, 1);
        }
    }
}

