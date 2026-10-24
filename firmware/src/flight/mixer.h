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

// Quad-X 混控函数
// 输入：
//   throttle —— 油门，范围 0.0 ~ 1.0
//   roll     —— 横滚修正量（来自 roll 轴 PID）
//   pitch    —— 俯仰修正量（来自 pitch 轴 PID）
//   yaw      —— 偏航修正量（来自 yaw 轴 PID）
// 输出：
//   out      —— 4 个电机的占空比，范围 0.0 ~ 1.0
//
// 注意：符号约定以实机验证为准。
// 如果装机后发现某个轴纠偏方向反了，把该修正量整体取反即可。
static inline void mixer_quad_x(float throttle, float roll, float pitch, float yaw,
                                MotorOutput_t *out) {
    // M1 右前 (CCW)：roll 加、pitch 减、yaw 加
    out->m1 = clampf(throttle - pitch + roll + yaw, 0.0f, 1.0f);
    // M2 右后 (CW)：roll 加、pitch 加、yaw 减
    out->m2 = clampf(throttle + pitch + roll - yaw, 0.0f, 1.0f);
    // M3 左后 (CCW)：roll 减、pitch 加、yaw 加
    out->m3 = clampf(throttle + pitch - roll + yaw, 0.0f, 1.0f);
    // M4 左前 (CW)：roll 减、pitch 减、yaw 减
    out->m4 = clampf(throttle - pitch - roll - yaw, 0.0f, 1.0f);
}