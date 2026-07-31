#include "line_sensor.h"
#include "zf_driver_adc.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

#define AD2 B8
#define AD1 B9
#define AD0 B14

int adc_mode = 0;
int adc_raw_value[8] = { 0 };
int adc_background_value[8] = { 0 };
int adc_foreground_value[8] = { 0 };
int adc_calibrated_value[8] = { 0 };

float line_error_raw = 0.0f;
float line_error_filtered = 0.0f;

void adc_capture_init(void)
{
    gpio_init(AD2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(AD1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(AD0, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_set_level(AD2, 1);
    gpio_set_level(AD1, 1);
    gpio_set_level(AD0, 1);
}

void adc_scan(void)
{
    gpio_set_level(AD2, 0); gpio_set_level(AD1, 0); gpio_set_level(AD0, 0); system_delay_us(1);
    adc_raw_value[0] = adc_mean_filter_convert(ADC1_CH5_B18, 15);

    gpio_set_level(AD2, 0); gpio_set_level(AD1, 0); gpio_set_level(AD0, 1); system_delay_us(1);
    adc_raw_value[1] = adc_mean_filter_convert(ADC1_CH5_B18, 15);

    gpio_set_level(AD2, 0); gpio_set_level(AD1, 1); gpio_set_level(AD0, 0); system_delay_us(1);
    adc_raw_value[2] = adc_mean_filter_convert(ADC1_CH5_B18, 15);

    gpio_set_level(AD2, 0); gpio_set_level(AD1, 1); gpio_set_level(AD0, 1); system_delay_us(1);
    adc_raw_value[3] = adc_mean_filter_convert(ADC1_CH5_B18, 15);

    gpio_set_level(AD2, 1); gpio_set_level(AD1, 0); gpio_set_level(AD0, 0); system_delay_us(1);
    adc_raw_value[4] = adc_mean_filter_convert(ADC1_CH5_B18, 15);

    gpio_set_level(AD2, 1); gpio_set_level(AD1, 0); gpio_set_level(AD0, 1); system_delay_us(1);
    adc_raw_value[5] = adc_mean_filter_convert(ADC1_CH5_B18, 15);

    gpio_set_level(AD2, 1); gpio_set_level(AD1, 1); gpio_set_level(AD0, 0); system_delay_us(1);
    adc_raw_value[6] = adc_mean_filter_convert(ADC1_CH5_B18, 15);

    gpio_set_level(AD2, 1); gpio_set_level(AD1, 1); gpio_set_level(AD0, 1); system_delay_us(1);
    adc_raw_value[7] = adc_mean_filter_convert(ADC1_CH5_B18, 15);
}

void adc_capture(void)
{
    adc_scan();
    if (adc_mode == 0) {
        for (int i = 0; i < 8; i++) {
            int diff = adc_foreground_value[i] - adc_background_value[i];
            if (diff == 0) diff = 1;
            adc_calibrated_value[i] = (adc_raw_value[i] - adc_background_value[i]) * 100 / diff;
            if (adc_calibrated_value[i] < 0) adc_calibrated_value[i] = 0;
            if (adc_calibrated_value[i] > 100) adc_calibrated_value[i] = 100;
        }
    }
    else if (adc_mode == 1) {
        for (int i = 0; i < 8; i++) adc_background_value[i] = adc_raw_value[i];
    }
    else if (adc_mode == 2) {
        for (int i = 0; i < 8; i++) adc_foreground_value[i] = adc_raw_value[i];
    }
}

void calculate_line_error(void)
{
    float cha_L = 0.0f, cha_R = 0.0f;
    float he_L = 0.0f, he_R = 0.0f;

    he_L = adc_calibrated_value[0] + adc_calibrated_value[1] + adc_calibrated_value[2] + adc_calibrated_value[3];
    he_R = adc_calibrated_value[4] + adc_calibrated_value[5] + adc_calibrated_value[6] + adc_calibrated_value[7];

    cha_L = (adc_calibrated_value[0] * 5) + (adc_calibrated_value[1] * 4) + (adc_calibrated_value[2] * 3) + adc_calibrated_value[3];
    cha_R = (adc_calibrated_value[7] * 5) + (adc_calibrated_value[6] * 4) + (adc_calibrated_value[5] * 3) + adc_calibrated_value[4];

    float denominator = he_L + he_R;
    if (denominator > 10.0f)
        line_error_raw = (cha_L - cha_R) * 350.0f / denominator;
    else
        line_error_raw = 0.0f;

    if (line_error_raw > 500) line_error_raw = 500;
    if (line_error_raw < -500) line_error_raw = -500;

    float err_scaled = line_error_raw * 0.007f;
    line_error_filtered = 0.84f * line_error_filtered + 0.16f * err_scaled;
}
