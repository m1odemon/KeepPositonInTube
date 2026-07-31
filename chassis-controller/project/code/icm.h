#ifndef ICM_H
#define ICM_H
#include "zf_common_headfile.h"
// ICM function declarations go here

#define Ang2Rad 0.01745329252f //角度制转弧度制
#define Rad2Ang 57.295779513f  //弧度制转角度制


typedef enum
{
    BIQUAD_LOWPASS,//低通滤波器（允许频率低于某一特定值（即截止频率）的信号通过，而频率高于该值的信号会被衰减）
    BIQUAD_HIGHPASS,//高通滤波器
    BIQUAD_BANDPASS_PEAK,//带通滤波器（允许位于两个特定频率之间的信号通过，限制范围之外所有频段），峰值类型配置
    BIQUAD_BANDSTOP_NOTCH,//带通滤波器（创建某个特定的频率范围，阻止该范围内的信号通过）
}biquad_type;

typedef struct
{
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
}biquad_state;
extern float Pitch_a, Roll_a, Yaw_a;
extern float Pitch_g, Roll_g, Yaw_g;   //实时角速度
extern float Pitch_g_F, Roll_g_F, Yaw_g_F;
extern volatile float Yaw_TotalAngle;   //Yaw总角度
extern float Yaw_AngleLast;             //上一次的Yaw角度
extern int Yaw_RoundCount;              //Yaw圈数计数
extern biquad_state adc_error;
void get_ICM_data(void);
void Filter_Init(void);
float biquad(biquad_state *state, float data);
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);

#endif // ICM_H

