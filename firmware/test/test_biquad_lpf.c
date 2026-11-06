#include <stdio.h>
#include <math.h>
// 1. 引用我们写好的二阶巴特沃斯低通滤波器头文件
#include "../src/flight/biquad_lpf.h"

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] %s (line %d)\n", msg, __LINE__); \
        g_failures++; \
    } else { \
        printf("[PASS] %s\n", msg); \
    } \
} while (0)

int main(void) {
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
    float last_out = 0.0f;
    for (int step = 1; step <= 25; step++) {
        last_out = biquad_update(&lpf, 10.0f);
        if (step == 1 || step % 5 == 0) {
            printf("Step %2d | Raw Input: 10.00 | Filtered Output: %6.2f\n", step, last_out);
        }
    }
    CHECK(fabsf(last_out - 10.0f) < 0.05f, "step response converges to 10.0 within 25 ticks");

    // ====================================================================
    // 场景 2：真动作（5Hz 慢速倾斜）+ 假动作（150Hz 电机剧烈震动）混合测试
    // ====================================================================
    biquad_init(&lpf, sample_freq, cutoff_freq);
    printf("\n========== Test 2: 5Hz Real Signal + 150Hz Motor Noise ==========\n");
    printf("Step | True(5Hz) | Raw(With Noise) | Filtered(LPF)\n");
    printf("-----+-----------+-----------------+--------------\n");

    float max_tracking_err = 0.0f;
    for (int step = 0; step < 40; step++) {
        float t = step * dt;
        float true_signal = 5.0f * sinf(2.0f * (float)M_PI * 5.0f * t);
        float motor_noise = 10.0f * sinf(2.0f * (float)M_PI * 150.0f * t);
        float raw_input = true_signal + motor_noise;
        float filtered = biquad_update(&lpf, raw_input);

        if (step >= 20) {
            printf(" %2d  |  %6.2f   |     %7.2f     |    %6.2f\n",
                   step, true_signal, raw_input, filtered);
            float err = fabsf(filtered - true_signal);
            if (err > max_tracking_err) {
                max_tracking_err = err;
            }
        }
    }
    CHECK(max_tracking_err < 2.5f, "150Hz motor noise attenuated while preserving 5Hz signal");

    printf("=================================================================\n");
    if (g_failures == 0) {
        printf("ALL BIQUAD LPF TESTS PASSED!\n");
        return 0;
    }
    printf("BIQUAD LPF TESTS FAILED: %d\n", g_failures);
    return 1;
}
