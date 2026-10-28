/*
    物理痛点：
    我们的8520空心杯电机高速运转时，转速很大，机架会产生高频抖动。
    直接传入传感器数，放大这种抖动，导致电机损伤，祸及MOS管。

    解决思路：
    为了让传感器数据更平滑，降低高频抖动的影响，我们可以在传感器数据输入到控制算法之前，先对其进行低通滤波处理。
    用‘低通滤波器’来改良这种影响,拦截剧烈跳动的高频震动，允许低频穿过。
    
    我们采用400HZ的采样频率与40HZ的截止频率，设计一个二阶低通滤波器，来对传感器数据进行处理。
    每秒读取400次传感器数据，凡是不超过40HZ的震动都可以通过滤波器，而超过40HZ的震动则会被滤掉，从而达到平滑传感器数据的目的。

*/

#pragma once

#include <math.h>

// 如果标准库没有定义圆周率 M_PI，我们手动补上
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 1. 定义二阶低通滤波器的“状态盒子”：
// 作用：保存滤波系数（决定筛子孔径大小）和前两拍的历史数据
typedef struct {
    // 5 个滤波器系数（由采样频率和截止频率共同决定）
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    // 4 个历史记忆槽：用来存前两拍的输入(x)和输出(y)
    float x1; // 上一拍的原始输入 x[n-1]
    float x2; // 上两拍的原始输入 x[n-2]
    float y1; // 上一拍的滤波输出 y[n-1]
    float y2; // 上两拍的滤波输出 y[n-2]
} BiquadLPF_t;

// 2. 滤波器初始化函数（开机时调用一次）：
// 输入：sample_freq 采样频率（如 400Hz），cutoff_freq 截止频率（如 40Hz）
static inline void biquad_init(BiquadLPF_t *lpf, float sample_freq, float cutoff_freq) {
    // 利用双线性变换公式，计算二阶巴特沃斯（Butterworth）系数
    // 其中 Q = 1 / sqrt(2) ≈ 0.70710678f，保证通带最平坦、不产生额外谐振尖峰
    float omega = 2.0f * (float)M_PI * cutoff_freq / sample_freq;
    float sn    = sinf(omega);
    float cs    = cosf(omega);
    float alpha = sn / (2.0f * 0.70710678f); // Q = 0.7071 (巴特沃斯特性)

    float a0 = 1.0f + alpha;

    // 归一化后的 5 个核心系数
    lpf->b0 = ((1.0f - cs) * 0.5f) / a0;
    lpf->b1 = (1.0f - cs) / a0;
    lpf->b2 = ((1.0f - cs) * 0.5f) / a0;
    lpf->a1 = (-2.0f * cs) / a0;
    lpf->a2 = (1.0f - alpha) / a0;

    // 开机时清空历史记忆槽
    lpf->x1 = 0.0f;
    lpf->x2 = 0.0f;
    lpf->y1 = 0.0f;
    lpf->y2 = 0.0f;
}

// 3. 滤波器核心计算函数（每次循环 400Hz 调用一次）：
// 输入：当前传感器带噪原始值 input
// 输出：滤除高频震动后的平滑值 output
static inline float biquad_update(BiquadLPF_t *lpf, float input) {
    // 【步骤 1：套用二阶差分方程算出当前输出】
    float output = lpf->b0 * input
                 + lpf->b1 * lpf->x1
                 + lpf->b2 * lpf->x2
                 - lpf->a1 * lpf->y1
                 - lpf->a2 * lpf->y2;

    // 【步骤 2：滚动更新记忆槽（为下一拍做准备）】
    lpf->x2 = lpf->x1;
    lpf->x1 = input;
    lpf->y2 = lpf->y1;
    lpf->y1 = output;

    return output;
}


