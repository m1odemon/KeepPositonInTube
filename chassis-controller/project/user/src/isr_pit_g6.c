/*
 * Interrupt wrapper paired with main_pit_g6.c.
 * TIMA1 is left available for the B4/B5 servo PWM channels; TIMG6 performs
 * the former 5 ms chassis-control work.
 */
#include "isr.h"
#include "servo.h"
#include "task3_ball.h"

#define TIMA0_IRQHandler TIMA0_IRQHandler_unused
#define TIMA1_IRQHandler TIMA1_IRQHandler_unused
#define TIMG0_IRQHandler TIMG0_IRQHandler_unused
#define TIMG6_IRQHandler TIMG6_IRQHandler_unused
#define TIMG7_IRQHandler TIMG7_IRQHandler_unused
#define TIMG8_IRQHandler TIMG8_IRQHandler_unused
#define TIMG12_IRQHandler TIMG12_IRQHandler_unused
#include "isr.c"
#undef TIMA0_IRQHandler
#undef TIMA1_IRQHandler
#undef TIMG0_IRQHandler
#undef TIMG6_IRQHandler
#undef TIMG7_IRQHandler
#undef TIMG8_IRQHandler
#undef TIMG12_IRQHandler

static void pit_g_dispatch_callback(uint8 pit_index)
{
    if (pit_callback_list[pit_index] != NULL) {
        pit_callback_list[pit_index](0, pit_callback_ptr_list[pit_index]);
    }
}

void TIMA0_IRQHandler(void)
{
    if (pit_callback_list[0] != NULL) {
        pit_callback_list[0](0, pit_callback_ptr_list[0]);
    }

    key_scanner();
    beep_actuator();
    servo_task8_update_5ms();
    task3_ball_update_5ms();
    servo_update_5ms();
    DL_TimerA_clearInterruptStatus(TIMA0, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void TIMA1_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMA1, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void TIMG0_IRQHandler(void)
{
    pit_g_dispatch_callback(2U);
    DL_TimerG_clearInterruptStatus(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void TIMG6_IRQHandler(void)
{
    pit_g_dispatch_callback(3U);

    get_ICM_data();
    tracking_control_loop();
    DL_TimerG_clearInterruptStatus(TIMG6, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void TIMG7_IRQHandler(void)
{
    pit_g_dispatch_callback(4U);
    DL_TimerG_clearInterruptStatus(TIMG7, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void TIMG8_IRQHandler(void)
{
    pit_g_dispatch_callback(5U);
    DL_TimerG_clearInterruptStatus(TIMG8, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void TIMG12_IRQHandler(void)
{
    pit_g_dispatch_callback(6U);
    DL_TimerG_clearInterruptStatus(TIMG12, DL_TIMER_INTERRUPT_ZERO_EVENT);
}
