/*
    Mahony 四元数姿态解算
    why?为什么要去算--->飞控的PID需要知道无人机现在歪了几度(EulerAngle_t)
                        MPU6050 传感器只会告诉你无人机的加速度和角速度，飞控需要知道无人机的姿态角度才能去控制无人机
    how?怎么去算--->通过陀螺仪和加速度计的传感器数据，经过数学运算，得到无人机的姿态角度
*/
#pragma once

#include <math.h>
#include "types.h" // 复用我们定义的 Vector3_t 和 EulerAngle_t

// 加速度计可信重力模长窗口（要求输入 accel 单位为 [g]，静止时模长约为 1.0g）
// 超出 [0.5g, 1.5g] 说明飞机处于剧烈加减速或撞击中，此时暂停加速度计修正，仅靠陀螺仪积分
#define MAHONY_ACCEL_MIN_G  0.5f
#define MAHONY_ACCEL_MAX_G  1.5f

// 陀螺仪零偏积分上限（单位 [rad/s]，0.10 rad/s ≈ 5.7 deg/s，足以覆盖 MPU6050 温漂且防积分发散）
#define MAHONY_BIAS_LIMIT   0.10f

// 内部辅助限幅函数（命名为 mahony_clampf，避免与 mixer.h 中的 clampf 重名冲突）
static inline float mahony_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 1. 定义 Mahony 姿态解算器的“状态盒子”
typedef struct {
    float kp;         // 比例增益 [无量纲/s 权重]：决定有多相信加速度计（拉回水平的速度，典型值 2.0f）
    float ki;         // 积分增益：用于慢慢消除陀螺仪的静态零偏（典型值 0.005f）

    // 单位四元数：内部维护的三维姿态（开机水平静止时为 1, 0, 0, 0）
    float q0;
    float q1;
    float q2;
    float q3;

    // 三轴积分误差累加器：专门记住并抵消陀螺仪的温漂/零偏，单位 [rad/s]
    float ix;
    float iy;
    float iz;
} Mahony_t;

// 2. 状态复位函数（将四元数重置为水平单位四元数，清空零偏积分）
static inline void mahony_reset(Mahony_t *m) {
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;

    m->ix = 0.0f;
    m->iy = 0.0f;
    m->iz = 0.0f;
}

// 3. 初始化函数（开机调用一次）
static inline void mahony_init(Mahony_t *m, float kp, float ki) {
    m->kp = kp;
    m->ki = ki;
    mahony_reset(m);
}

// 4. Mahony 六轴姿态更新核心函数（每次控制循环 400Hz 调用一次）
// 输入：
//   gyro  —— 滤波后的三轴角速度（必须为弧度/秒 [rad/s]，FRD 坐标系）
//   accel —— 滤波后的三轴比力加速度（单位必须为 [g]，静止平放时约为 {0, 0, +1.0g}）
//   dt    —— 采样周期 [s]（例如 400Hz 对应 0.0025f）
static inline void mahony_update_imu(Mahony_t *m, Vector3_t gyro, Vector3_t accel, float dt) {
    float gx = gyro.x;
    float gy = gyro.y;
    float gz = gyro.z;

    float ax = accel.x;
    float ay = accel.y;
    float az = accel.z;

    // 【步骤 1：检查加速度计模长是否在可信重力窗口 (0.5g ~ 1.5g) 内】
    float norm_a = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_a > MAHONY_ACCEL_MIN_G && norm_a < MAHONY_ACCEL_MAX_G) {
        // 将实测加速度归一化为单位向量
        float inv_a = 1.0f / norm_a;
        ax *= inv_a;
        ay *= inv_a;
        az *= inv_a;

        // 【步骤 2：从当前四元数推算“飞控心里以为的重力方向 v”】
        float vx = 2.0f * (m->q1 * m->q3 - m->q0 * m->q2);
        float vy = 2.0f * (m->q0 * m->q1 + m->q2 * m->q3);
        float vz = m->q0 * m->q0 - m->q1 * m->q1 - m->q2 * m->q2 + m->q3 * m->q3;

        // 【步骤 3：向量叉乘 e = a × v，算出实测重力与估计重力的方向偏差】
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // 【步骤 4：更新 I 项（学习陀螺仪零偏并限幅）与 P 项（比例拉回）】
        if (m->ki > 0.0f && dt > 0.0f) {
            m->ix = mahony_clampf(m->ix + m->ki * ex * dt, -MAHONY_BIAS_LIMIT, MAHONY_BIAS_LIMIT);
            m->iy = mahony_clampf(m->iy + m->ki * ey * dt, -MAHONY_BIAS_LIMIT, MAHONY_BIAS_LIMIT);
            m->iz = mahony_clampf(m->iz + m->ki * ez * dt, -MAHONY_BIAS_LIMIT, MAHONY_BIAS_LIMIT);
        }

        // P 项：仅在加速度计可信时才按比例把姿态拽向重力方向
        gx += m->kp * ex;
        gy += m->kp * ey;
        gz += m->kp * ez;
    }

    // 【关键！】无论当前这一拍加速度计是否超重，已经学到的陀螺仪零偏 (ix, iy, iz) 必须始终补偿！
    gx += m->ix;
    gy += m->iy;
    gz += m->iz;

    // 【步骤 5：用修正后的角速度积分更新四元数（一阶微分方程：dq/dt = 0.5 * q ⊗ ω）】
    float half_dt = 0.5f * dt;
    float q0 = m->q0;
    float q1 = m->q1;
    float q2 = m->q2;
    float q3 = m->q3;

    m->q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    m->q1 += ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    m->q2 += ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    m->q3 += ( q0 * gz + q1 * gy - q2 * gx) * half_dt;

    // 【步骤 6：四元数重新归一化（防止浮点运算误差积累导致长度偏离 1）】
    float norm_q = sqrtf(m->q0 * m->q0 + m->q1 * m->q1 + m->q2 * m->q2 + m->q3 * m->q3);
    if (norm_q > 1e-6f) {
        float inv_q = 1.0f / norm_q;
        m->q0 *= inv_q;
        m->q1 *= inv_q;
        m->q2 *= inv_q;
        m->q3 *= inv_q;
    }
}

// 5. 将四元数翻译为人类和 PID 看得懂的欧拉角（输出为弧度制 [rad]，对齐 types.h FRD 约定）
static inline void mahony_to_euler(const Mahony_t *m, EulerAngle_t *out) {
    float q0 = m->q0;
    float q1 = m->q1;
    float q2 = m->q2;
    float q3 = m->q3;

    // Roll（横滚角，绕 +X 轴）
    out->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // Pitch（俯仰角，绕 +Y 轴）：先限幅到 [-1, 1]，防止浮点误差导致 asinf 算出 NaN
    float sinp = mahony_clampf(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f);
    out->pitch = asinf(sinp);

    // Yaw（航向角，绕 +Z 轴）
    out->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
