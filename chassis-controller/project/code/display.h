#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "zf_common_headfile.h"
#include "battery_3s.h"


typedef struct {
    char name[20];      // 参数名称
    float *value;       // 参数值指针
    float step;         // 调整步长
    float min_val;      // 最小值
    float max_val;      // 最大值
    uint8_t is_int;     // 是否为整数参数
} param_item_t;

// ---------------------------------------------------------
// UI 与 菜单全局变量声明 (使用 extern 暴露给其他文件)
// ---------------------------------------------------------
extern param_item_t param_list[];

extern uint8_t param_edit_mode;
extern uint8_t selected_param;
extern uint8_t param_count;

extern float speed_scale;

/*
 * H题行驶时间接口。
 *
 * 当前仅提供独立函数，尚未接入主函数、按键状态机、5 ms中断或 show_ui()。
 * 后续接入时：按键发车调用 start，5 ms控制周期调用 update_5ms，
 * 完成停车调用 stop，主循环选择合适页面调用 show。
 */
#define H_DRIVE_TIMER_READY    (0U)
#define H_DRIVE_TIMER_RUNNING  (1U)
#define H_DRIVE_TIMER_STOPPED  (2U)

void h_drive_timer_reset(void);
void h_drive_timer_start(void);
void h_drive_timer_update_5ms(void);
void h_drive_timer_stop(void);
uint32_t h_drive_timer_get_ms(void);
uint8_t h_drive_timer_get_state(void);
void h_drive_timer_show(uint8_t row);
void h_drive_timer_show_state(uint8_t row);

// 兼容 main.c 当前调用的空接口
int assign_value(void);

// ---------------------------------------------------------
// 对外接口函数声明
// ---------------------------------------------------------
// 极速逻辑处理 (供 isr.c 中的 5ms 中断调用)
void menu_control(void);

// 慢速屏幕渲染 (供 main.c 中的 while(1) 调用)
void show_ui(void);

#endif
