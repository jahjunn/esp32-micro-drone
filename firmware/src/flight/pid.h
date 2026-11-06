#pragma once
#include <stdbool.h>

// 1. 定义 PID 控制器的“状态盒子”：
// 作用：保存算法计算所需的所有参数、历史状态和限幅值
typedef struct {
    float kp;             // P 参数：当前误差增益
    float ki;             // I 参数：过去积累误差增益
    float kd;             // D 参数：未来趋势阻尼增益
    float integral;       // 积分累加池：用来存累计的历史误差
    float prev_measured;  // 记忆槽：记录上一拍的【真实测量值】（用于测量值微分，防打杆尖峰）
    bool  has_prev;       // 首拍标志：false 表示刚开机或刚复位，第一拍不计算 D 项尖峰
    float integral_limit; // 积分池上限：防积分饱和（Anti-windup）
    float output_limit;   // 最终输出上限：防电机控制量超负荷
} PID_t;

// 2. PID 状态复位函数：
// 作用：在飞机上锁 (LOCKED)、失控 (FAILSAFE) 或怠速未起飞时调用，清空历史积分与微分记忆
static inline void pid_reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_measured = 0.0f;
    pid->has_prev = false;
}

// 3. PID 初始化函数：
// 作用：给参数赋初始值，并调用 pid_reset 清空内部状态
static inline void pid_init(PID_t *pid, float kp, float ki, float kd, float int_limit, float out_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = int_limit;
    pid->output_limit = out_limit;
    pid_reset(pid);
}

static inline float pid_update_measurement(PID_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // 【步骤 1：算 P 项】
    float p_term = pid->kp * error;

    // 【步骤 2：算 D 项（测量值微分：-d(measured)/dt，消除打杆突变与首拍尖峰）】
    float derivative = 0.0f;
    if (pid->has_prev && dt > 0.0f) {
        derivative = -(measured - pid->prev_measured) / dt;
    }
    float d_term = pid->kd * derivative;
    pid->prev_measured = measured;
    pid->has_prev = true;

    // 【步骤 3：算 I 项（带条件抗积分饱和 Conditional Anti-Windup）】
    // 若当前输出已经顶满上限且误差仍为正（或顶满下限且误差仍为负），停止向饱和方向继续累加积分！
    if (dt > 0.0f && pid->ki != 0.0f) {
        float pre_output = p_term + (pid->ki * pid->integral) + d_term;
        bool saturating_high = (pre_output >=  pid->output_limit) && (error > 0.0f);
        bool saturating_low  = (pre_output <= -pid->output_limit) && (error < 0.0f);
        if (!saturating_high && !saturating_low) {
            pid->integral += error * dt;
            if (pid->integral > pid->integral_limit) {
                pid->integral = pid->integral_limit;
            } else if (pid->integral < -pid->integral_limit) {
                pid->integral = -pid->integral_limit;
            }
        }
    }
    float i_term = pid->ki * pid->integral;

    // 【步骤 4：求和并做总输出限幅】
    float output = p_term + i_term + d_term;
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}

// 5. 兼容单参数误差接口（等价于 setpoint = 0, measured = -error）：
static inline float pid_update(PID_t *pid, float error, float dt) {
    return pid_update_measurement(pid, 0.0f, -error, dt);
}