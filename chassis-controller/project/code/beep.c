#include "beep.h"
#include "zf_common_headfile.h"

/* The extension board buzzer is connected to PA14. */
#define BEEP_PIN A14

static volatile int beep_time = 0;
static int beep_T = 1;

void beep_init(int tick_ms)
{
    gpio_init(BEEP_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    beep_T = (tick_ms > 0) ? tick_ms : 1;
}

void beep_on(void)
{
    gpio_set_level(BEEP_PIN, GPIO_HIGH);
}

void beep_off(void)
{
    gpio_set_level(BEEP_PIN, GPIO_LOW);
}

void beep_set_time(int time_ms)
{
    if (beep_T <= 0) {
        beep_time = 0;
        return;
    }

    beep_time = time_ms / beep_T;
}

void beep_actuator(void)
{
    if (beep_time > 0) {
        beep_on();
        beep_time--;
    }
    else {
        beep_off();
    }
}
