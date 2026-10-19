// 1. 头文件守卫：防止这个头文件被多个文件同时引用时造成“重复定义”报错
#pragma once

// 2. 引入 C 语言标准库中的基础类型：包含 uint8_t, int16_t, bool 等
#include <stdint.h>
#include <stdbool.h>

// 3. 定义通用三维向量：
// 作用：打包 X, Y, Z 三个轴的数据，给传感器（加速度计/陀螺仪）统一格式
typedef struct {
    float x; // X轴数值
    float y; // Y轴数值
    float z; // Z轴数值
} Vector3_t;

// 4. 定义姿态欧拉角：
// 作用：飞控经过数学姿态解算后输出的“机身倾角”（弧度制）
typedef struct {
    float roll;      // 横滚角：左右倾斜程度（向右歪为正）
    float pitch;     // 俯仰角：前后翘起程度（机头翘为正）
    float yaw;       // 航向角：机头指向东西南北的哪个角度
} EulerAngle_t;

// 5. 定义遥控器指令帧：
// 作用：把手机滑动摇杆发送过来的网络数据，打包转化为飞控理解的控制目标
typedef struct {
    float throttle;         // 油门大小（0.0 代表关机，1.0 代表 100% 满推力）
    float roll_target;      // 飞手期望飞机达到的横滚角度
    float pitch_target;     // 飞手期望飞机达到的俯仰角度
    float yaw_rate_target;  // 飞手期望飞机原地旋转的速度
    bool  armed;            // 安全锁：true=已解锁允许电机转；false=锁定，哪怕推油门也不转
} ControlCommand_t;

// 6. 定义电机混控输出：
// 作用：把姿态修正量分配后，最终下发给 4 个电机的 PWM 占空比比例（0.0~1.0）
typedef struct {
    float m1; // 右前电机 (逆时针 CCW)
    float m2; // 右后电机 (顺时针 CW)
    float m3; // 左后电机 (逆时针 CCW)
    float m4; // 左前电机 (顺时针 CW)
} MotorOutput_t;