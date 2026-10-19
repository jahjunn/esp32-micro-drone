#pragma once

// 1. 定义 PID 控制器的“状态盒子”：
// 作用：保存算法计算所需的所有参数、历史误差和限幅值
typedef struct {
    float kp;             // P 参数：当前误差增益
    float ki;             // I 参数：过去积累误差增益
    float kd;             // D 参数：未来趋势预测增益
    float integral;       // 积分累加池：用来存累计的历史误差
    float prev_error;     // 记忆槽：记录上一拍的误差，用来算变化率
    float integral_limit; // 积分池上限：防饱和（救命参数）
    float output_limit;   // 最终输出上限：防电机超负荷
} PID_t;

// 2. PID 初始化函数：
// 作用：给参数赋初始值，将积分池和历史误差清零
static inline void pid_init(PID_t *pid, float kp, float ki, float kd, float int_limit, float out_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;       // 开机时没有任何历史累积
    pid->prev_error = 0.0f;     // 开机时没有历史误差
    pid->integral_limit = int_limit;
    pid->output_limit = out_limit;
}

// 3. PID 核心计算函数（每次循环调用一次）：
// 输入：当前误差 error（目标角度 - 实际角度），时间间隔 dt（秒）
// 输出：应该给电机施加的纠偏力矩 output
static inline float pid_update(PID_t *pid, float error, float dt) {
    
    // 【步骤 1：算 P 项】
    // 作用：眼前的偏差有多大，就按比例输出多大力气
    float p_term = pid->kp * error;

    // 【步骤 2：算 I 项（加限幅）】
    // 作用：把误差乘以时间累计到积分池中
    pid->integral += error * dt;
    // 强制限幅：如果累积超出安全上限，强行截断，防止未起飞时积分过大
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->ki * pid->integral;

    // 【步骤 3：算 D 项】
    // 作用：计算误差在单位时间内的变化速度（斜率），起阻尼刹车作用，防超调晃动
    float derivative = (dt > 0.0f) ? ((error - pid->prev_error) / dt) : 0.0f;
    float d_term = pid->kd * derivative;

    // 更新记忆槽：把当前误差记下来，下一拍计算时作为“上一拍历史误差”
    pid->prev_error = error;

    // 【步骤 4：求和并做总输出限幅】
    // 作用：三项相加得到最终纠偏指令，但绝不能超出电机硬件承受范围
    float output = p_term + i_term + d_term;
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}