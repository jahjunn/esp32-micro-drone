/*
    test_mahony.c 三个测试场景在验什么？
    场景 1：静止抗温漂测试（验证“静止不发散” + Ki 消零偏）
    模拟险情：飞机明明平放在桌上不动（加速度计 Z = 1g），但陀螺仪坏脾气犯了，一直谎报自己每秒在往右转 3°/s（模拟真实传感器的零点温漂）。
    预期现象：如果只靠陀螺仪，2 秒钟就歪了 6°；但在 Mahony 下，刚开始只会被骗不到 1°，随后加速度计和积分项 ix 迅速识破谎言，把姿态死死按回 0.00°！
    场景 2：大角度倾斜收敛与回平测试（验证“晃动后能收敛” + Kp 拉回）
    模拟动作：假装飞机突然被手掰歪到 Roll = +30° 静置（此时重力分了一半到 Y 轴：ay = sin(30°) = 0.5，az = cos(30°) ≈ 0.866），随后再放回水平桌面。
    预期现象：解算出的 Roll 会迅速收敛并稳定在 30.00°；放平后又能平滑归零到 0.00°。
    场景 3：动态跟手测试（验证“陀螺仪+加速度计配合零延迟”）
    模拟动作：飞机以 60°/s 的角速度匀速抬头（绕 Y 轴 Pitch）转 0.5 秒。
    预期现象：0.5 秒 结束的那一瞬间，解算出的 Pitch 分毫不差停在 30.00°，没有任何迟钝拖影！
    场景 4: 大机动超重抗干扰测试。验证当猛推油门产生横向1.5g+纵向1.5g的加速度时，Mahony不会被干扰，不盲信加速度计，仍然能稳定输出正确姿态。
*/
#include <stdio.h>
#include <math.h>
#include "../src/flight/mahony.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD(d) ((d) * ((float)M_PI / 180.0f))
#define RAD2DEG(r) ((r) * (180.0f / (float)M_PI))

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
    Mahony_t ahrs;
    EulerAngle_t euler;
    float dt = 1.0f / 400.0f;

    // 场景 1：静止抗漂移测试（陀螺仪 X 轴存在 3 deg/s 假零偏）
    mahony_init(&ahrs, 2.0f, 0.5f);
    printf("====== Test 1: Stationary Anti-Drift (Gyro Bias = 3 deg/s) ======\n");
    Vector3_t bias_gyro  = { DEG2RAD(3.0f), 0.0f, 0.0f };
    Vector3_t flat_accel = { 0.0f, 0.0f, 1.0f };

    for (int step = 1; step <= 4800; step++) {
        mahony_update_imu(&ahrs, bias_gyro, flat_accel, dt);
        if (step % 400 == 0) {
            mahony_to_euler(&ahrs, &euler);
            printf("Time %d.0s | Roll: %6.2f deg | Learned Bias(ix): %6.2f deg/s\n",
                   step / 400, RAD2DEG(euler.roll), RAD2DEG(ahrs.ix));
        }
    }
    mahony_to_euler(&ahrs, &euler);
    CHECK(fabsf(RAD2DEG(euler.roll)) < 0.2f, "Test 1: stationary roll drift suppressed < 0.2 deg");
    CHECK(fabsf(RAD2DEG(ahrs.ix) - (-3.0f)) < 0.2f, "Test 1: integral learned -3.0 deg/s gyro bias");

    // 场景 2：倾斜收敛测试（机身向右歪 30 度静置）
    mahony_init(&ahrs, 2.0f, 0.005f);
    printf("\n====== Test 2: Tilt Convergence (Static Roll = +30.0 deg) ======\n");
    Vector3_t zero_gyro  = { 0.0f, 0.0f, 0.0f };
    Vector3_t tilt_accel = { 0.0f, sinf(DEG2RAD(30.0f)), cosf(DEG2RAD(30.0f)) };

    for (int step = 1; step <= 1200; step++) {
        mahony_update_imu(&ahrs, zero_gyro, tilt_accel, dt);
        if (step % 200 == 0) {
            mahony_to_euler(&ahrs, &euler);
            printf("Time %.1fs | Roll: %6.2f deg | Pitch: %6.2f deg | Yaw: %6.2f deg\n",
                   step * dt, RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));
        }
    }
    mahony_to_euler(&ahrs, &euler);
    CHECK(fabsf(RAD2DEG(euler.roll) - 30.0f) < 0.5f, "Test 2: static tilt converges to +30.0 deg");

    // 场景 3：动态跟手测试（以 60 deg/s 匀速抬头 0.5 秒 -> 30.0 deg）
    mahony_init(&ahrs, 2.0f, 0.005f);
    printf("\n====== Test 3: Dynamic Tracking (Pitching at 60 deg/s for 0.5s) ======\n");
    float pitch_rate = DEG2RAD(60.0f);

    for (int step = 1; step <= 200; step++) {
        float true_pitch = pitch_rate * (step * dt);
        Vector3_t dyn_gyro  = { 0.0f, pitch_rate, 0.0f };
        Vector3_t dyn_accel = { -sinf(true_pitch), 0.0f, cosf(true_pitch) };
        mahony_update_imu(&ahrs, dyn_gyro, dyn_accel, dt);

        if (step % 50 == 0) {
            mahony_to_euler(&ahrs, &euler);
            printf("Step %3d (%.3fs) | True Pitch: %6.2f deg | Mahony Pitch: %6.2f deg\n",
                   step, step * dt, RAD2DEG(true_pitch), RAD2DEG(euler.pitch));
        }
    }
    mahony_to_euler(&ahrs, &euler);
    CHECK(fabsf(RAD2DEG(euler.pitch) - 30.0f) < 0.2f, "Test 3: dynamic pitch tracks to +30.0 deg");

    // 场景 4：大机动超重拒信测试（norm_a = sqrt(1.5^2 + 1.5^2) = 2.12g > 1.5g）
    // 飞机真实角速度为 0（保持水平），但加速度计因剧烈横向加速读到 {0, 1.5g, 1.5g}（歪 45 度）
    // 预期：因为 2.12g 超出 [0.5g, 1.5g]，Mahony 必须拒绝采信，Roll 保持 0.00 deg 不被带偏！
    mahony_init(&ahrs, 2.0f, 0.005f);
    printf("\n====== Test 4: High-G Maneuver Rejection (norm_a = 2.12g) ======\n");
    Vector3_t high_g_accel = { 0.0f, 1.5f, 1.5f };
    for (int step = 1; step <= 200; step++) {
        mahony_update_imu(&ahrs, zero_gyro, high_g_accel, dt);
    }
    mahony_to_euler(&ahrs, &euler);
    printf("After 0.5s of 2.12g lateral accel | Roll: %6.2f deg\n", RAD2DEG(euler.roll));
    CHECK(fabsf(RAD2DEG(euler.roll)) < 1e-4f, "Test 4: >1.5g accel is rejected, roll stays 0.0 deg");

    printf("======================================================================\n");
    if (g_failures == 0) {
        printf("ALL MAHONY TESTS PASSED!\n");
        return 0;
    }
    printf("MAHONY TESTS FAILED: %d\n", g_failures);
    return 1;
}