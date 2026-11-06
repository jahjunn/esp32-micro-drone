#include <stdio.h>
#include <math.h>
#include "../src/flight/pid.h"

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
    PID_t pid;
    //合理的阻尼系数，避免高频抽搐
    pid_init(&pid, 2.0f, 0.5f, 0.02f, 10.0f, 20.0f);

    float current_angle = 10.0f;  // 初始被风吹偏 10 度
    float target_angle  = 0.0f;   // 目标保持 0 度水平
    float dt = 0.01f;             // 10ms 控制周期

    printf("================ PID Simulation Test ================\n");

    float first_output = 0.0f;
    for (int step = 1; step <= 15; step++) {
        float error = target_angle - current_angle;
        float output = pid_update_measurement(&pid, target_angle, current_angle, dt);
        if (step == 1) {
            first_output = output;
        }
        printf("Step %2d | Angle: %6.2f deg | Error: %6.2f | Output: %6.2f\n",
               step, current_angle, error, output);
        current_angle += output * 0.15f;
    }

    // 断言 1：第一拍不应触发 D 项尖峰（P=-20, I=-0.05，没有 -100 的 D 项）
    CHECK(pid.has_prev == true, "first tick sets has_prev = true");
    CHECK(fabsf(first_output - (-20.0f)) < 0.01f, "first tick output is smooth without D-kick");

    // 断言 2：15 拍后角度平滑收敛到接近 0 度（误差小于 0.5 度，且无剧烈反向超调）
    CHECK(fabsf(current_angle) < 0.5f, "closed-loop angle converges smoothly near 0.0 deg");

    // 断言 3：测试打杆阶跃（Setpoint Step）不产生 D 项踹击
    pid_reset(&pid);
    CHECK(pid.has_prev == false && fabsf(pid.integral) < 1e-6f, "pid_reset clears integral and has_prev");

    // 先在 setpoint=0, measured=0 跑一拍建立历史记忆
    (void)pid_update_measurement(&pid, 0.0f, 0.0f, dt);
    // 下一拍摇杆突然打到 setpoint=5.0，但飞机真实姿态 measured 还是 0.0
    // 此时只有 P=2.0*5.0=10.0, I=0.5*(5.0*0.01)=0.025, D 必须为 0（因为 measured 没变！）
    float step_out = pid_update_measurement(&pid, 5.0f, 0.0f, dt);
    CHECK(fabsf(step_out - 10.025f) < 1e-4f, "setpoint step produces zero derivative kick");

    printf("==================================================\n");
    if (g_failures == 0) {
        printf("ALL PID TESTS PASSED!\n");
        return 0;
    }
    printf("PID TESTS FAILED: %d\n", g_failures);
    return 1;
}