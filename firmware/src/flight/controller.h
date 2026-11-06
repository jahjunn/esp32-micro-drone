/*
    controller.h —— 飞控 400Hz 串级控制总指挥
    核心防线：
    1. NaN/Inf 传感器脏数据防火墙（防 I2C 干扰导致 IIR 滤波器与四元数永久中毒）
    2. 陀螺仪开机 200 拍静止校准 + 手抖峰峰值超限自动重测 + 未校准禁止解锁
    3. 电池电压一阶低通滤波（防 PWM 大电流尖峰误触发 LOWBAT_LAND 单向锁存）
    4. 地面怠速静默机制（解锁但油门 <= 5% 时均匀输出 12% 怠速并清积分，防桌面跳舞与 D 项丢失）
    5. 80Hz 陀螺仪 LPF + 40Hz 加速度计 LPF + 400Hz Mahony + 100Hz 外环角度 P + 400Hz 内环角速度 PID
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "types.h"
#include "biquad_lpf.h"
#include "mahony.h"
#include "pid.h"
#include "mixer.h"
#include "safety.h"

// 控制环路频率与滤波参数定义
#define CTRL_LOOP_HZ              400.0f
#define CTRL_DT                   (1.0f / CTRL_LOOP_HZ) // 0.0025s (2.5ms)
#define CTRL_OUTER_DIVIDER        4U                    // 外环每 4 拍更新 1 次 (100Hz)
#define CTRL_GYRO_LPF_CUTOFF_HZ   80.0f                 // 陀螺仪二阶巴特沃斯截止频率：80Hz
#define CTRL_ACCEL_LPF_CUTOFF_HZ  40.0f                 // 加速度计二阶巴特沃斯截止频率：40Hz
#define CTRL_VBAT_LPF_ALPHA       0.02f                 // 电池电压一阶滤波系数（防瞬间电流毛刺误锁存）

// 陀螺仪开机静止校准参数
#define CTRL_GYRO_CAL_SAMPLES     200U                  // 静止采样 200 拍（400Hz 下为 0.5 秒）
#define CTRL_GYRO_CAL_MOTION_RAD  0.15f                 // 校准期间最大允许抖动峰峰值（0.15 rad/s ≈ 8.6°/s）

typedef struct {
    // 传感器三轴二阶巴特沃斯低通滤波器（0:X, 1:Y, 2:Z）
    BiquadLPF_t  gyro_lpf[3];
    BiquadLPF_t  accel_lpf[3];

    // 陀螺仪静态零偏与在线静止校准状态机
    Vector3_t    gyro_bias;
    bool         gyro_calibrated;
    uint16_t     cal_count;
    Vector3_t    cal_sum;
    Vector3_t    cal_min;
    Vector3_t    cal_max;

    // 滤波后的安全电池电压（V）
    float        vbat_filtered;

    // Mahony 姿态解算器与当前欧拉角（FRD，弧度制）
    Mahony_t     ahrs;
    EulerAngle_t attitude;

    // 安全状态机
    Safety_t     safety;

    // 外环 100Hz 角度 P 控制器（输出目标角速度 rad/s）
    PID_t        angle_roll_pid;
    PID_t        angle_pitch_pid;
    Vector3_t    desired_rate;

    // 内环 400Hz 角速度 PID 控制器（输出混控力矩）
    PID_t        rate_roll_pid;
    PID_t        rate_pitch_pid;
    PID_t        rate_yaw_pid;

    // 节拍计数器
    uint32_t     tick;
} FlightController_t;

// 内部辅助：检查三维向量是否全部为合法有限浮点数（防 NaN / Inf 污染滤波器）
static inline bool controller_vec3_is_finite(Vector3_t v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// 清空全部内外环 PID 积分与微分历史
static inline void controller_reset_pids(FlightController_t *fc) {
    pid_reset(&fc->angle_roll_pid);
    pid_reset(&fc->angle_pitch_pid);
    pid_reset(&fc->rate_roll_pid);
    pid_reset(&fc->rate_pitch_pid);
    pid_reset(&fc->rate_yaw_pid);
    fc->desired_rate.x = 0.0f;
    fc->desired_rate.y = 0.0f;
    fc->desired_rate.z = 0.0f;
}

// 重置陀螺仪校准累加器（开机或检测到手抖时调用）
static inline void controller_reset_gyro_cal(FlightController_t *fc) {
    fc->cal_count = 0U;
    fc->cal_sum.x = 0.0f;
    fc->cal_sum.y = 0.0f;
    fc->cal_sum.z = 0.0f;
}

// 飞控总指挥初始化
static inline void controller_init(FlightController_t *fc) {
    for (int i = 0; i < 3; ++i) {
        biquad_init(&fc->gyro_lpf[i],  CTRL_LOOP_HZ, CTRL_GYRO_LPF_CUTOFF_HZ);
        biquad_init(&fc->accel_lpf[i], CTRL_LOOP_HZ, CTRL_ACCEL_LPF_CUTOFF_HZ);
    }

    fc->gyro_bias.x     = 0.0f;
    fc->gyro_bias.y     = 0.0f;
    fc->gyro_bias.z     = 0.0f;
    fc->gyro_calibrated = false;
    controller_reset_gyro_cal(fc);

    fc->vbat_filtered   = 0.0f;

    mahony_init(&fc->ahrs, 2.0f, 0.005f);
    fc->attitude.roll   = 0.0f;
    fc->attitude.pitch  = 0.0f;
    fc->attitude.yaw    = 0.0f;

    safety_init(&fc->safety);

    // 外环 100Hz 角度 P 参数（1 rad 误差 -> 4.5 rad/s 目标角速度，上限 ±5.0 rad/s）
    pid_init(&fc->angle_roll_pid,  4.5f, 0.0f, 0.0f, 0.0f, 5.0f);
    pid_init(&fc->angle_pitch_pid, 4.5f, 0.0f, 0.0f, 0.0f, 5.0f);

    // 内环 400Hz 角速度 PID 参数
       // 内环 400Hz 角速度 PID 参数（内环带宽约为外环角度环的 3 倍，消除相位滞后超调）
    pid_init(&fc->rate_roll_pid,   0.25f, 0.08f, 0.004f, 0.20f, 0.40f);
    pid_init(&fc->rate_pitch_pid,  0.25f, 0.08f, 0.004f, 0.20f, 0.40f);
    pid_init(&fc->rate_yaw_pid,    0.25f, 0.10f, 0.000f, 0.20f, 0.30f);

    controller_reset_pids(fc);
    fc->tick = 0U;
}

// 手动直接指定陀螺仪零偏并标记已校准（供桌面单元测试或预存校准值使用）
static inline void controller_set_gyro_bias(FlightController_t *fc, Vector3_t bias_frd) {
    fc->gyro_bias       = bias_frd;
    fc->gyro_calibrated = true;
}

// 陀螺仪开机静止自动校准步进函数（带手抖峰峰值检测，抖动超限自动清零重采）
// 返回值：true 表示已完成 200 拍静止校准，false 表示仍在采集中
static inline bool controller_calibrate_gyro_sample(FlightController_t *fc, Vector3_t raw_gyro_frd) {
    if (fc->gyro_calibrated) {
        return true;
    }
    if (!controller_vec3_is_finite(raw_gyro_frd)) {
        return false;
    }

    if (fc->cal_count == 0U) {
        fc->cal_min = raw_gyro_frd;
        fc->cal_max = raw_gyro_frd;
    } else {
        if (raw_gyro_frd.x < fc->cal_min.x) fc->cal_min.x = raw_gyro_frd.x;
        if (raw_gyro_frd.y < fc->cal_min.y) fc->cal_min.y = raw_gyro_frd.y;
        if (raw_gyro_frd.z < fc->cal_min.z) fc->cal_min.z = raw_gyro_frd.z;
        if (raw_gyro_frd.x > fc->cal_max.x) fc->cal_max.x = raw_gyro_frd.x;
        if (raw_gyro_frd.y > fc->cal_max.y) fc->cal_max.y = raw_gyro_frd.y;
        if (raw_gyro_frd.z > fc->cal_max.z) fc->cal_max.z = raw_gyro_frd.z;

        // 如果采样期间机身被晃动（任一轴峰峰值 > 0.15 rad/s），丢弃重来！
        if ((fc->cal_max.x - fc->cal_min.x) > CTRL_GYRO_CAL_MOTION_RAD ||
            (fc->cal_max.y - fc->cal_min.y) > CTRL_GYRO_CAL_MOTION_RAD ||
            (fc->cal_max.z - fc->cal_min.z) > CTRL_GYRO_CAL_MOTION_RAD) {
            controller_reset_gyro_cal(fc);
            return false;
        }
    }

    fc->cal_sum.x += raw_gyro_frd.x;
    fc->cal_sum.y += raw_gyro_frd.y;
    fc->cal_sum.z += raw_gyro_frd.z;
    fc->cal_count++;

    if (fc->cal_count >= CTRL_GYRO_CAL_SAMPLES) {
        float inv_n = 1.0f / (float)CTRL_GYRO_CAL_SAMPLES;
        fc->gyro_bias.x     = fc->cal_sum.x * inv_n;
        fc->gyro_bias.y     = fc->cal_sum.y * inv_n;
        fc->gyro_bias.z     = fc->cal_sum.z * inv_n;
        fc->gyro_calibrated = true;
        return true;
    }
    return false;
}

// 将正面朝上贴装的 MPU6050 芯片原始读数（FLU: +X前, +Y左, +Z上）转换为飞控标准 FRD 右手系
static inline void controller_map_flu_to_frd(Vector3_t gyro_flu,
                                             Vector3_t accel_flu,
                                             Vector3_t *gyro_frd,
                                             Vector3_t *accel_frd) {
    gyro_frd->x  =  gyro_flu.x;
    gyro_frd->y  = -gyro_flu.y;
    gyro_frd->z  = -gyro_flu.z;

    accel_frd->x = -accel_flu.x;
    accel_frd->y =  accel_flu.y;
    accel_frd->z =  accel_flu.z;
}

// 飞控 400Hz 单拍核心步进函数
static inline FlightState_t controller_step(FlightController_t *fc,
                                            Vector3_t raw_gyro_frd,
                                            Vector3_t raw_accel_frd,
                                            ControlCommand_t cmd,
                                            float vbat_v,
                                            uint32_t cmd_age_ms,
                                            MotorOutput_t *out_motors) {
    // 【防线 1：NaN / Inf 脏传感器数据拦截】
    if (!controller_vec3_is_finite(raw_gyro_frd) || !controller_vec3_is_finite(raw_accel_frd)) {
        raw_gyro_frd  = fc->gyro_bias;
        raw_accel_frd = (Vector3_t){0.0f, 0.0f, 1.0f};
    }

    // 【防线 2：如果陀螺仪尚未完成开机静止校准，自动喂入校准器并禁止解锁】
    if (!fc->gyro_calibrated) {
        controller_calibrate_gyro_sample(fc, raw_gyro_frd);
        cmd.armed = false;
    }

    // 【防线 3：电池电压一阶低通滤波（防大电流脉冲毛刺误触发 LOWBAT_LAND 锁存）】
    if (isfinite(vbat_v) && vbat_v > 0.1f) {
        if (fc->vbat_filtered <= 0.1f) {
            fc->vbat_filtered = vbat_v;
        } else {
            fc->vbat_filtered += CTRL_VBAT_LPF_ALPHA * (vbat_v - fc->vbat_filtered);
        }
    }

        // 开机第 1 拍：直接用首帧加速度预充电 LPF 记忆槽，消除从 0g 爬升到 1g 的冷启动阶跃延迟
    if (fc->tick == 0U) {
        biquad_reset_to(&fc->accel_lpf[0], raw_accel_frd.x);
        biquad_reset_to(&fc->accel_lpf[1], raw_accel_frd.y);
        biquad_reset_to(&fc->accel_lpf[2], raw_accel_frd.z);
    }

    // 【步骤 1：扣除陀螺仪静态零偏，并分别过 80Hz (Gyro) 与 40Hz (Accel) 二阶巴特沃斯低通滤波器】
    Vector3_t gyro_filt = {
        biquad_update(&fc->gyro_lpf[0], raw_gyro_frd.x - fc->gyro_bias.x),
        biquad_update(&fc->gyro_lpf[1], raw_gyro_frd.y - fc->gyro_bias.y),
        biquad_update(&fc->gyro_lpf[2], raw_gyro_frd.z - fc->gyro_bias.z)
    };
    Vector3_t accel_filt = {
        biquad_update(&fc->accel_lpf[0], raw_accel_frd.x),
        biquad_update(&fc->accel_lpf[1], raw_accel_frd.y),
        biquad_update(&fc->accel_lpf[2], raw_accel_frd.z)
    };

    // 【步骤 2：400Hz 更新 Mahony 姿态解算】
    // 地面上锁阶段用高增益 (Kp=10.0) 快速对齐桌面真实水平；解锁飞行后切回标准增益 (Kp=2.0) 抗加减速干扰
    fc->ahrs.kp = (fc->safety.state == FLIGHT_STATE_LOCKED) ? 10.0f : 2.0f;
    mahony_update_imu(&fc->ahrs, gyro_filt, accel_filt, CTRL_DT);
    mahony_to_euler(&fc->ahrs, &fc->attitude);

    // 【步骤 3：过安全状态机（传入滤波后的稳定电池电压）】
    FlightState_t state = safety_update(&fc->safety, &cmd, fc->attitude, fc->vbat_filtered, cmd_age_ms);

    // 【步骤 4：上锁或故障急停态，清空 PID 并将四路电机彻底焊死为 0】
    if (state == FLIGHT_STATE_LOCKED || state == FLIGHT_STATE_FAILSAFE) {
        controller_reset_pids(fc);
        out_motors->m1 = 0.0f;
        out_motors->m2 = 0.0f;
        out_motors->m3 = 0.0f;
        out_motors->m4 = 0.0f;
        fc->tick++;
        return state;
    }

    // 【防线 4：地面怠速静默态（已解锁但油门 <= 5%）】
    if (cmd.throttle <= SAFETY_ARM_MAX_THROTTLE) {
        controller_reset_pids(fc);
        out_motors->m1 = MOTOR_IDLE_DEFAULT;
        out_motors->m2 = MOTOR_IDLE_DEFAULT;
        out_motors->m3 = MOTOR_IDLE_DEFAULT;
        out_motors->m4 = MOTOR_IDLE_DEFAULT;
        fc->tick++;
        return state;
    }

    // 【步骤 5：外环 100Hz 角度 P 控制（刚离开怠速的首拍立即执行，随后每 4 拍更新一次）】
    if (!fc->angle_roll_pid.has_prev || (fc->tick % CTRL_OUTER_DIVIDER) == 0U) {
        float outer_dt = CTRL_DT * (float)CTRL_OUTER_DIVIDER; // 0.01s
        fc->desired_rate.x = pid_update_measurement(&fc->angle_roll_pid,
                                                    cmd.roll_target,
                                                    fc->attitude.roll,
                                                    outer_dt);
        fc->desired_rate.y = pid_update_measurement(&fc->angle_pitch_pid,
                                                    cmd.pitch_target,
                                                    fc->attitude.pitch,
                                                    outer_dt);
    }

    // 【步骤 6：内环 400Hz 角速度 PID 控制】
    float roll_torque  = pid_update_measurement(&fc->rate_roll_pid,
                                                fc->desired_rate.x,
                                                gyro_filt.x,
                                                CTRL_DT);
    float pitch_torque = pid_update_measurement(&fc->rate_pitch_pid,
                                                fc->desired_rate.y,
                                                gyro_filt.y,
                                                CTRL_DT);
    float yaw_torque   = pid_update_measurement(&fc->rate_yaw_pid,
                                                fc->desired_rate.z,
                                                gyro_filt.z,
                                                CTRL_DT);

    // 【步骤 7：Quad-X 混控 + 12% 怠速保底映射 + 电机安全总闸门】
    mixer_quad_x(cmd.throttle, roll_torque, pitch_torque, yaw_torque, out_motors);
    mixer_apply_idle(out_motors, MOTOR_IDLE_DEFAULT);
    safety_apply_motor_gate(&fc->safety, out_motors);

    fc->tick++;
    return state;
}