#pragma once // pragma once 头文件守卫，防止重复引用报错
#include "types.h" // 作用：引用我们上面定义的 MotorOutput_t 结构体类型

/*
 PID只算出了三个修正量：
 roll : 左右倾斜需要纠正多少
 pitch : 前后翘起需要纠正多少
 yaw : 机头旋转需要纠正多少
 加上throttle油门，此时有四个量 --->> 飞机有四个电机，怎么分配？
 将四个修正量分配给四个电机的各自转速与比例，这就是混控算法的作用
 */

/*
    我们做一个简单的四电机飞行器模型
        电机	位置	转向
         M1	    右前	CCW
         M2  	右后	CW
         M3	    左后	CCW
         M4	    左前	CW

对角线的两个电机转向相同，这样它们产生的反扭矩互相抵消，机身不会自己旋转

*/


//        机头
//   M4(CW)    M1(CCW)
//        ╲   ╱
//         ╳
//        ╱   ╲
//   M3(CCW)   M2(CW)

// 限幅函数 clampf ；将数值限制在上下限之间
// 把v限制在[lo,hi]之间，如果v小于lo就返回lo，大于hi就返回hi，否则返回v本身
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 保均值抗饱和函数（借鉴 CF-Drone 的 Desaturation 算法）
// 作用：当某个电机超出 [0.0, 1.0] 时，保持 4 个电机的平均总推力不变，
//       等比例缩放各电机相对于平均值的偏差，防止大油门打杆时飞机掉高
static inline void mixer_desaturate(MotorOutput_t *out) {
    // 1. 算出当前 4 个电机的平均推力（并确保平均值本身在 0.0 ~ 1.0 之间）
    float avg = clampf((out->m1 + out->m2 + out->m3 + out->m4) * 0.25f, 0.0f, 1.0f);

    // 2. 找出 4 个电机里的最大值 max_v 和最小值 min_v
    float max_v = out->m1;
    if (out->m2 > max_v) max_v = out->m2;
    if (out->m3 > max_v) max_v = out->m3;
    if (out->m4 > max_v) max_v = out->m4;

    float min_v = out->m1;
    if (out->m2 < min_v) min_v = out->m2;
    if (out->m3 < min_v) min_v = out->m3;
    if (out->m4 < min_v) min_v = out->m4;

    // 3. 计算等比例压缩系数 scale（默认 1.0 代表没超限，不需要压缩）
    float scale = 1.0f;

    // 如果最大值冲破了上限 1.0，计算需要把向上的偏差压缩多少倍
    if (max_v > 1.0f && max_v > avg) {
        float s_high = (1.0f - avg) / (max_v - avg);
        if (s_high < scale) scale = s_high;
    }

    // 如果最小值跌破了下限 0.0，计算需要把向下的偏差压缩多少倍
    if (min_v < 0.0f && avg > min_v) {
        float s_low = avg / (avg - min_v);
        if (s_low < scale) scale = s_low;
    }

    // 4. 如果发生了超限（scale < 1.0），围绕平均推力 avg 等比缩放 4 个电机
    if (scale < 1.0f) {
        out->m1 = avg + (out->m1 - avg) * scale;
        out->m2 = avg + (out->m2 - avg) * scale;
        out->m3 = avg + (out->m3 - avg) * scale;
        out->m4 = avg + (out->m4 - avg) * scale;
    }

    // 5. 最后用 clampf 兜底消除极微小的浮点数舍入误差
    out->m1 = clampf(out->m1, 0.0f, 1.0f);
    out->m2 = clampf(out->m2, 0.0f, 1.0f);
    out->m3 = clampf(out->m3, 0.0f, 1.0f);
    out->m4 = clampf(out->m4, 0.0f, 1.0f);
}

// Quad-X 混控函数
// 8520 空心杯默认怠速占空比（12%）：
// 作用：解锁后保持电机微转，防止大机动抗饱和把某侧电机压到 0 停转（停转再起转需 100~200ms 会导致翻机）
#define MOTOR_IDLE_DEFAULT 0.12f

// Quad-X 混控函数（严格遵循 FRD 正方向契约）：
//   - roll  > 0（需要向右滚转力矩）-> 左侧电机(M3/M4)加速(+)，右侧电机(M1/M2)减速(-)
//   - pitch > 0（需要抬头力矩）    -> 前侧电机(M1/M4)加速(+)，后侧电机(M2/M3)减速(-)
//   - yaw   > 0（需要顺时针右转）  -> 逆时针桨(M1/M3)加速(+)，顺时针桨(M2/M4)减速(-)
static inline void mixer_quad_x(float throttle, float roll, float pitch, float yaw,
                                MotorOutput_t *out) {
    // 1. 先按线性混控公式算出未截断的原始分配值
    // M1 右前 (CCW)：抬头加(+pitch)、右倾减(-roll)、右转加(+yaw)
    out->m1 = throttle + pitch - roll + yaw;
    // M2 右后 (CW) ：抬头减(-pitch)、右倾减(-roll)、右转减(-yaw)
    out->m2 = throttle - pitch - roll - yaw;
    // M3 左后 (CCW)：抬头减(-pitch)、右倾加(+roll)、右转加(+yaw)
    out->m3 = throttle - pitch + roll + yaw;
    // M4 左前 (CW) ：抬头加(+pitch)、右倾加(+roll)、右转减(-yaw)
    out->m4 = throttle + pitch + roll - yaw;

    // 2. 进行保均值抗饱和处理（防止单边饱和导致掉高）
    mixer_desaturate(out);
}

// 电机怠速保底映射函数（必须在 mixer_quad_x 之后、safety_apply_motor_gate 之前调用！）
// 作用：将 [0.0, 1.0] 的混控输出线性映射到 [idle, 1.0]，确保解锁飞行中任何电机绝不停转
static inline void mixer_apply_idle(MotorOutput_t *out, float idle) {
    float cl_idle = clampf(idle, 0.0f, 0.30f); // 安全钳位：怠速最高不超过 30%
    float span    = 1.0f - cl_idle;
    out->m1 = cl_idle + clampf(out->m1, 0.0f, 1.0f) * span;
    out->m2 = cl_idle + clampf(out->m2, 0.0f, 1.0f) * span;
    out->m3 = cl_idle + clampf(out->m3, 0.0f, 1.0f) * span;
    out->m4 = cl_idle + clampf(out->m4, 0.0f, 1.0f) * span;

}
