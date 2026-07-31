#include "icm.h"
#include <stdint.h>
#include <math.h>
//-------------------------------------------------------------------------------------------------------------------
// 本文件是关于姿态解算的相关内容
//-------------------------------------------------------------------------------------------------------------------
#ifndef PI
#define PI 3.141592654
#endif
#define sampleFreq  200.0f    //采样频率（赫兹）
#define twoKpDef    (2.0f * 0.5f)
#define twoKiDef    (2.0f * 0.0f)
/*
 * twoKpDef,比例增益（当前为1）调整系统响应速度
 * p越高，误差的快速反应越强，但导致动态性能变差（超调）
 *  twoKiDef，积分增益（目前为0），调整长期或慢速变化趋势的纠正能力
 */

volatile float invsampleFreq = 1.0f / sampleFreq;
volatile float twoKp = twoKpDef;
volatile float twoKi = twoKiDef;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                  // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
/* 角度 */
float Roll_A0, Pitch_A0, Yaw_A0;
float Pitch_a, Roll_a, Yaw_a;   //实时姿态角
float Pitch_a_Pi, Roll_a_Pi, Roll_error_Pi, Yaw_a_Pi, Yaw_a_Pi_genrl, Yaw_a_Pi_local;
                                //取模角度值，确保在指定范围（0到π或全周期）内
float Pitch_a_last, Roll_a_last, Yaw_a_last;
float aim_roll_a,aim_pitch_a,aim_yaw_a,d_aim_yaw_a;
volatile float Yaw_TotalAngle = 0.0f;   //Yaw总角度
float Yaw_AngleLast = 0.0f;     //上一次的Yaw角度
int Yaw_RoundCount = 0;         //Yaw圈数计数器
/* (角)速度 */
float Pitch_g, Roll_g, Yaw_g;   //实时角速度
float Pitch_g_F, Roll_g_F, Yaw_g_F;
                                //滤波角速度
float Pitch_g_F_last, Roll_g_F_last,Yaw_g_F_last;
                                // 上一次的滤波角速度

float Pitch_g_last, Roll_g_last, Yaw_g_last;
                                //上一周期的角速度，用作参考或历史数据点
//float aim_x_v,aim_y_v,aim_y_v_last,aim_yaw_g;
float aim_x_v,aim_y_v,aim_y_v_last;
                                //目标角速度
/* (线)速度 */
float X_V,Y_V;                  //线速度分量
float X_V_R_last,Y_V_R_last;    //前一周期速度数据（相对于某个参考点的速度）
float X_V_last,Y_V_last;        //上一个时间间隔的线速度值
float X_V_F,Y_V_F;              //滤波线速度读数
/* 加速度 */
                                //加速度
float Pitch_acc, Roll_acc, Yaw_acc;
float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;
                                //过滤加速度
biquad_state Pitch_g_biquad, Roll_g_biquad, Yaw_g_biquad,
             Pitch_acc_biquad, Roll_acc_biquad, Yaw_acc_biquad,Roll_g_biquad2,adc_error;

void get_ICM_data(void)
{
    // gpio_set_level(B26, 1); // B26 作为测试用GPIO
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    // gpio_set_level(B26, 0); // B26 作为测试用GPIO
    //加速度存原始值
    Pitch_acc = -icm42688_acc_y;
    Roll_acc  =  icm42688_acc_x;
    Yaw_acc   =  icm42688_acc_z;
    //二阶滤波对加速度进行修饰
    Pitch_acc_F = biquad(&Pitch_acc_biquad,Pitch_acc);
    Roll_acc_F  = biquad(&Roll_acc_biquad,Roll_acc);
    Yaw_acc_F   = biquad(&Yaw_acc_biquad,Yaw_acc);
    //角速度存原始值
    Pitch_g = -icm42688_gyro_y;
    Roll_g  =  icm42688_gyro_x;
    Yaw_g   =  icm42688_gyro_z;
    //二阶滤波对角速度进行修饰
    Pitch_g_F = - biquad(&Pitch_g_biquad,Pitch_g);
    Roll_g_F  = biquad(&Roll_g_biquad,Roll_g);
    Yaw_g_F   = biquad(&Yaw_g_biquad,Yaw_g);
    // gpio_set_level(B26, 1); // B26 作为测试用GPIO
    MahonyAHRSupdateIMU(icm42688_gyro_x,icm42688_gyro_y,icm42688_gyro_z,icm42688_acc_x,icm42688_acc_y,icm42688_acc_z);
    // gpio_set_level(B26, 0); // B26 作为测试用GPIO
    Pitch_a  = - Rad2Ang * Pitch_a_Pi;//实时姿态角
    Roll_a   = Rad2Ang * Roll_a_Pi;
    Yaw_a    = Rad2Ang * Yaw_a_Pi;
    
    // 计算Yaw总角度，处理超过360度的情况
    if (Yaw_a - Yaw_AngleLast > 180.0f)
    {
        Yaw_RoundCount--;
    }
    else if (Yaw_a - Yaw_AngleLast < -180.0f)
    {
        Yaw_RoundCount++;
    }
    Yaw_TotalAngle = 360.0f * Yaw_RoundCount + Yaw_a;
    Yaw_AngleLast = Yaw_a;
}

void Filter_Init(void)
{
    biquad_filter_init(&Pitch_g_biquad, BIQUAD_LOWPASS, 200, 30, 0.7071);
    biquad_filter_init(&Roll_g_biquad, BIQUAD_LOWPASS, 200, 20, 0.5773);
    biquad_filter_init(&Yaw_g_biquad, BIQUAD_LOWPASS, 200, 30, 0.5773);
    biquad_filter_init(&Pitch_acc_biquad, BIQUAD_LOWPASS, 200, 10, 0.5773);
    biquad_filter_init(&Roll_acc_biquad, BIQUAD_LOWPASS, 200, 10, 0.5773);
    biquad_filter_init(&Yaw_acc_biquad, BIQUAD_LOWPASS, 200, 10, 0.5773);
    biquad_filter_init(&Roll_g_biquad2, BIQUAD_LOWPASS, 200, 20, 0.7071);
    biquad_filter_init(&adc_error, BIQUAD_LOWPASS, 200, 10, 0.5773);
}
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    // 定义静态变量，用于存储计算结果和状态。静态变量在函数每次调用时会保持其值
    static float recipNorm;
    static float halfvx, halfvy, halfvz;
    static float halfex, halfey, halfez;//航向角速度
    static float qa, qb, qc;
    //判断的用途，当运动发生才进行计算
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        //角速度转换成弧度，单位为rad/s
        gx*=0.0174532925f;
        gy*=0.0174532925f;
        gz*=0.0174532925f;

        // 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向和垂直于磁通量的矢量
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 误差是估计重力方向与测量重力方向的乘积之和
        // 计算欧拉角估计与实际测量的交叉积，作为误差项
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 如果启用，计算并应用积分反馈
        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invsampleFreq;  // 积分误差按Ki缩放
            integralFBy += twoKi * halfey * invsampleFreq;
            integralFBz += twoKi * halfez * invsampleFreq;
            gx += integralFBx;  // 应用积分反馈
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // 防止积分饱和
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 应用p反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    // 计算四元数的时间积分，时间增量是通过乘以采样频率的一半来计算的
    gx *= (0.5f * invsampleFreq);       // 预乘常用因子
    gy *= (0.5f * invsampleFreq);
    gz *= (0.5f * invsampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 计算四元数的模长来归一化四元数（确保其满足单位四元数条件）
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    static float r11,r12,r21,r31,r32;
    // 计算旋转矩阵的元素
    r11 = 2.0f * (q0*q1 + q2*q3);
    r12 = 1.0f - 2.0f * (q1*q1 + q2*q2);
    r21 = 2.0f * (q0*q2 - q3*q1);
    r31 = 2.0f * (q0*q3 + q1*q2);
    r32 = 1.0f - 2.0f * (q2*q2 + q3*q3);

    // 计算欧拉角(弧度)
    Yaw_a_Pi = atan2f(r31, r32);
    Pitch_a_Pi = -asinf(r21);
    Roll_a_Pi = atan2f(r11, r12);
}
//二阶（Biquad）滤波器配置类型选择

// 2nd-order Butterworth: q_value=0.7071（最均匀的选择性，意味着相位失真最小）
// 2nd-order Chebyshev (ripple 1 dB): q_value=0.9565（允许在特定频率范围内有一定的波动，但频带外的衰减非常快。用在对阻带衰减有严格要求）
// 2nd-order Thomson-Bessel: q_value=0.5773（良好的时间域响应和相位线性度，低延迟和良好相位响应的系统中使用）
// 带通滤波器的q_value = sqrt(f1*f2)/(f2-f1)
//后三个参数是采样率（赫兹）、原始截止频率（赫兹）、调整滤波器品质因数的值
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value)
{
    float w0, sin_w0, cos_w0, alpha;
    float b0, b1, b2, a0, a1, a2;

    w0 = 2 * PI * fc / fs;//频率转换
    sin_w0 = sinf(w0);
    cos_w0 = cosf(w0);
    alpha = sin_w0 / (2.0 * q_value);//衰减系数

    switch(type)
    {
    case BIQUAD_LOWPASS:
        b0 = (1.0 - cos_w0) / 2.0;
        b1 = b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_HIGHPASS:
        b0 = (1.0 + cos_w0) / 2.0;
        b1 = -b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDPASS_PEAK:
        b0 = alpha;
        b1 = 0.0;
        b2 = -alpha;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDSTOP_NOTCH:
        b0 = 1.0;
        b1 = -2.0 * cos_w0;
        b2 = 1.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    }
    state->a0 = b0 / a0;
    state->a1 = b1 / a0;
    state->a2 = b2 / a0;
    state->a3 = a1 / a0;
    state->a4 = a2 / a0;
    state->x1 = state->x2 = 0.0;
    state->y1 = state->y2 = 0.0;
}

//二阶滤波
float biquad(biquad_state *state, float data)
{
    float result = 0;
    result = state->a0 * data + state->a1 * state->x1 + state->a2 * state->x2 -  state->a3 * state->y1 - state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = data;
    state->y2 = state->y1;
    state->y1 = result;
    return result;
}
// 快速平方根倒数计算
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
