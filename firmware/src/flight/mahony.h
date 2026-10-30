/*
    Mahony 四元数姿态解算
    why?为什么要去算--->飞控的PID需要知道无人机现在歪了几度(EulerAngle_t)
                        MPU6050 传感器只会告诉你无人机的加速度和角速度，飞控需要知道无人机的姿态角度才能去控制无人机
    how?怎么去算--->通过陀螺仪和加速度计的传感器数据，经过数学运算，得到无人机的姿态角度

    优点：（Mahony的关键点）
    1.猜测重力方向：根据飞控存储的实时数据，模拟出当前引力与无人机的方向
    2.与真实重力比较，得到真实误差：用实测的加速度方向与所推测的方向做叉乘
    3.PI修正角速度：kp和ki修正
            kp:比例拉回，将误差乘入陀螺仪角速度里，将姿态往真实水平修正
            ki：积分拉回，将误差积分后乘入陀螺仪角速度里，将姿态往真实水平修正
    4.积分陀螺仪角速度，得到四元数姿态：将修正后的陀螺仪角速度积分，得到四元数姿态
*/
#pragma once

#include <math.h>
#include "types.h" // 复用我们定义的 Vector3_t 和 EulerAngle_t

// 内部辅助限幅函数（命名为 mahony_clampf，避免与 mixer.h 中的 clampf 重名冲突）
static inline float mahony_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 1. 定义 Mahony 姿态解算器的“状态盒子”
typedef struct {
    float kp;         // 比例增益：决定有多相信加速度计（拉回水平的速度，典型值 2.0f）
    float ki;         // 积分增益：用于慢慢消除陀螺仪的静态零偏（典型值 0.005f）

    // 单位四元数：内部维护的三维姿态（开机水平静止时为 1, 0, 0, 0）
    float q0;
    float q1;
    float q2;
    float q3;

    // 三轴积分误差累加器：专门记住并抵消陀螺仪的温漂/零偏
    float ix;
    float iy;
    float iz;
} Mahony_t;

// 2. 初始化函数（开机调用一次）
static inline void mahony_init(Mahony_t *m, float kp, float ki) {
    m->kp = kp;
    m->ki = ki;

    // 初始姿态设为绝对水平、机头朝前的单位四元数 (1, 0, 0, 0)
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;

    // 清空零偏积分池
    m->ix = 0.0f;
    m->iy = 0.0f;
    m->iz = 0.0f;
}

// 3. Mahony 六轴姿态更新核心函数（每次控制循环 400Hz 调用一次）
// 输入：
//   gyro  —— 滤波后的三轴角速度（必须为弧度/秒 rad/s！）
//   accel —— 滤波后的三轴加速度（单位不限，g 或 m/s^2 均可，内部会归一化）
//   dt    —— 采样周期（秒，例如 400Hz 对应 0.0025f）
static inline void mahony_update_imu(Mahony_t *m, Vector3_t gyro, Vector3_t accel, float dt) {
    float gx = gyro.x;
    float gy = gyro.y;
    float gz = gyro.z;

    float ax = accel.x;
    float ay = accel.y;
    float az = accel.z;

    // 【步骤 1：检查加速度计数据是否有效（防止自由落体全 0 导致除以 0）】
    float norm_a = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_a > 1e-6f) {
        // 将实测加速度归一化为单位向量（只保留“真实正下方”的方向，不管长度）
        float inv_a = 1.0f / norm_a;
        ax *= inv_a;
        ay *= inv_a;
        az *= inv_a;

        // 【步骤 2：从当前四元数推算“飞控心里以为的重力方向 v”】
        // （即大地坐标系 Z 轴单位向量 [0,0,1] 旋转到机身坐标系后的方向）
        float vx = 2.0f * (m->q1 * m->q3 - m->q0 * m->q2);
        float vy = 2.0f * (m->q0 * m->q1 + m->q2 * m->q3);
        float vz = m->q0 * m->q0 - m->q1 * m->q1 - m->q2 * m->q2 + m->q3 * m->q3;

        // 【步骤 3：向量叉乘 e = a × v，算出实测重力与估计重力的方向偏差】
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // 【步骤 4：用 PI 控制器修正陀螺仪角速度】
        // I 项：累加误差以消除陀螺仪零偏
        if (m->ki > 0.0f) {
            m->ix += m->ki * ex * dt;
            m->iy += m->ki * ey * dt;
            m->iz += m->ki * ez * dt;
            gx += m->ix;
            gy += m->iy;
            gz += m->iz;
        }
        // P 项：按比例把姿态拽向真实重力方向
        gx += m->kp * ex;
        gy += m->kp * ey;
        gz += m->kp * ez;
    }

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

// 4. 将四元数翻译为人类和 PID 看得懂的欧拉角（输出为弧度制 rad，对齐 types.h 约定）
static inline void mahony_to_euler(const Mahony_t *m, EulerAngle_t *out) {
    float q0 = m->q0;
    float q1 = m->q1;
    float q2 = m->q2;
    float q3 = m->q3;

    // Roll（横滚角，绕 X 轴）
    out->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // Pitch（俯仰角，绕 Y 轴）：先限幅到 [-1, 1]，防止浮点误差导致 asinf 算出 NaN
    float sinp = mahony_clampf(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f);
    out->pitch = asinf(sinp);

    // Yaw（航向角，绕 Z 轴）
    out->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3));
}