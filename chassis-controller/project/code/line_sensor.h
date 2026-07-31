#ifndef _LINE_SENSOR_H_
#define _LINE_SENSOR_H_

/* 8-channel line sensor acquisition, calibration, and error estimation. */
extern int adc_mode;
extern int adc_raw_value[8];
extern int adc_background_value[8];
extern int adc_foreground_value[8];
extern int adc_calibrated_value[8];

extern float line_error_raw;
extern float line_error_filtered;

void adc_capture_init(void);
void adc_scan(void);
void adc_capture(void);
void calculate_line_error(void);

#endif
