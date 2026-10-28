#include <stdio.h>
#include <math.h>
// 1. 引用我们刚写好的二阶巴特沃斯低通滤波器头文件
#include "../src/flight/biquad_lpf.h"

int main() {
    BiquadLPF_t lpf;

    // 采样频率 400Hz（每拍 dt = 0.0025 秒），截止频率 40Hz
    float sample_freq = 400.0f;
    float cutoff_freq = 40.0f;
    float dt = 1.0f / sample_freq;

    // ====================================================================
    // 场景 1：阶跃响应测试（输入恒定为 10.0，验证稳态值是否精确收敛到 10.00）
    // ====================================================================
    biquad_init(&lpf, sample_freq, cutoff_freq);
    printf("========== Test 1: Step Response (Input = 10.0) ==========\n");
    for (int step = 1; step <= 25; step++) {
        float out = biquad_update(&lpf, 10.0f);
        // 每 5 拍打印一次，观察输出如何从 0 平滑爬升并稳定在 10.00
        if (step == 1 || step % 5 == 0) {
            printf("Step %2d | Raw Input: 10.00 | Filtered Output: %6.2f\n", step, out);
        }
    }

    // ====================================================================
    // 场景 2：真动作（5Hz 慢速倾斜）+ 假动作（150Hz 电机剧烈震动）混合测试
    // ====================================================================
    biquad_init(&lpf, sample_freq, cutoff_freq);
    printf("\n========== Test 2: 5Hz Real Signal + 150Hz Motor Noise ==========\n");
    printf("Step | True(5Hz) | Raw(With Noise) | Filtered(LPF)\n");
    printf("-----+-----------+-----------------+--------------\n");

    for (int step = 0; step < 40; step++) {
        float t = step * dt;

        // 真实机身动作：5Hz 正弦波，幅度 5.0（低于 40Hz，应该被放行）
        float true_signal = 5.0f * sinf(2.0f * (float)M_PI * 5.0f * t);

        // 电机高频震动：150Hz 正弦波，幅度高达 10.0（远高于 40Hz，应该被拦截）
        float motor_noise = 10.0f * sinf(2.0f * (float)M_PI * 150.0f * t);

        // 传感器实际读到的混合脏数据
        float raw_input = true_signal + motor_noise;

        // 送入我们的二阶巴特沃斯滤波器
        float filtered = biquad_update(&lpf, raw_input);

        // 从第 20 拍开始打印（此时滤波器已完成初始过渡，可清晰对比）
        if (step >= 20) {
            printf(" %2d  |  %6.2f   |     %7.2f     |    %6.2f\n",
                   step, true_signal, raw_input, filtered);
        }
    }
    printf("=================================================================\n");

    return 0;
}