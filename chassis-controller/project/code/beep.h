#ifndef _BEEP_H_
#define _BEEP_H_

/* Initialize the buzzer control period in milliseconds. */
void beep_init(int tick_ms);

/* Immediately drive the buzzer output high or low. */
void beep_on(void);
void beep_off(void);

/* Start a non-blocking buzzer interval in milliseconds. */
void beep_set_time(int time_ms);

/* Execute one buzzer timing step; call this from the periodic ISR. */
void beep_actuator(void);

#endif
