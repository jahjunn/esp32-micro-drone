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

*/
#include <stdio.h>
#include <math.h>
// 1. 引用我们刚写好的 Mahony 姿态解算头文件
#include "../src/flight/mahony.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 弧度 <-> 角度 转换宏（方便人类肉眼观察打印结果）
#define DEG2RAD(d) ((d) * ((float)M_PI / 180.0f))
#define RAD2DEG(r) ((r) * (180.0f / (float)M_PI))

int main() {
    Mahony_t ahrs;
    EulerAngle_t euler;

    // 控制频率 400Hz（每拍 dt = 0.0025 秒）
    float dt = 1.0f / 400.0f;

    // ====================================================================
    // 场景 1：静止抗漂移测试
    // 飞机水平静止（ax=0, ay=0, az=1g），但陀螺仪 X 轴存在 3 deg/s 的假零偏
    // 预期：姿态不会随时间无限漂移，而是被加速度计拉住并由 Ki 慢慢抵消零偏
    // ====================================================================
    mahony_init(&ahrs, 2.0f, 0.5f); // 适当调大 Ki 以便在几秒仿真内看清零偏抵消过程
    printf("====== Test 1: Stationary Anti-Drift (Gyro Bias = 3 deg/s) ======\n");
    Vector3_t bias_gyro  = { DEG2RAD(3.0f), 0.0f, 0.0f };
    Vector3_t flat_accel = { 0.0f, 0.0f, 1.0f };

    // 连续跑 2000 拍（模拟真实时间 5 秒）
    for (int step = 1; step <= 2000; step++) {
        mahony_update_imu(&ahrs, bias_gyro, flat_accel, dt);
        if (step % 400 == 0) { // 每满 1 秒打印一次
            mahony_to_euler(&ahrs, &euler);
            printf("Time %d.0s | Roll: %6.2f deg | Learned Bias(ix): %6.2f deg/s\n",
                   step / 400, RAD2DEG(euler.roll), RAD2DEG(ahrs.ix));
        }
    }

    // ====================================================================
    // 场景 2：倾斜收敛测试（机身向右歪 30 度静置 -> 观察能否收敛到 30.00 deg）
    // ====================================================================
    mahony_init(&ahrs, 2.0f, 0.005f);
    printf("\n====== Test 2: Tilt Convergence (Static Roll = +30.0 deg) ======\n");
    Vector3_t zero_gyro  = { 0.0f, 0.0f, 0.0f };
    // 机身绕 X 轴右倾 30° 时，1g 重力投影在 Y 轴为 sin(30°)=0.5，Z 轴为 cos(30°)≈0.866
    Vector3_t tilt_accel = { 0.0f, sinf(DEG2RAD(30.0f)), cosf(DEG2RAD(30.0f)) };

    // 连续跑 1200 拍（3 秒），每 0.5 秒（200 拍）打印一次
    for (int step = 1; step <= 1200; step++) {
        mahony_update_imu(&ahrs, zero_gyro, tilt_accel, dt);
        if (step % 200 == 0) {
            mahony_to_euler(&ahrs, &euler);
            printf("Time %.1fs | Roll: %6.2f deg | Pitch: %6.2f deg | Yaw: %6.2f deg\n",
                   step * dt, RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));
        }
    }

    // ====================================================================
    // 场景 3：动态跟手测试（以 60 deg/s 角速度匀速抬头旋转 0.5 秒）
    // 预期：0.5 秒（200 拍）结束时，Pitch 零延迟精确到达 30.00 deg
    // ====================================================================
    mahony_init(&ahrs, 2.0f, 0.005f);
    printf("\n====== Test 3: Dynamic Tracking (Pitching at 60 deg/s for 0.5s) ======\n");
    float pitch_rate = DEG2RAD(60.0f);

    for (int step = 1; step <= 200; step++) {
        float true_pitch = pitch_rate * (step * dt);
        Vector3_t dyn_gyro  = { 0.0f, pitch_rate, 0.0f };
        // 绕 Y 轴抬头 θ 时，重力在 X 轴投影为 -sin(θ)，Z 轴为 cos(θ)
        Vector3_t dyn_accel = { -sinf(true_pitch), 0.0f, cosf(true_pitch) };

        mahony_update_imu(&ahrs, dyn_gyro, dyn_accel, dt);

        if (step % 50 == 0) {
            mahony_to_euler(&ahrs, &euler);
            printf("Step %3d (%.3fs) | True Pitch: %6.2f deg | Mahony Pitch: %6.2f deg\n",
                   step, step * dt, RAD2DEG(true_pitch), RAD2DEG(euler.pitch));
        }
    }
    printf("======================================================================\n");

    return 0;
}