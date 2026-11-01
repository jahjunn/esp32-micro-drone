/*
    "safety.h"头文件创建的安全机制：

    1.地面解锁电压门槛提高到 3.50V（防带病起飞）： 1S 电池如果在地面空载时就已经低于 3.50V，4 个 8520 电机一起转（4~5A 大电流），电池电压会瞬间被拉垮到 3.0V 以下。所以地面解锁必须满足 vbat >= 3.50V；而在空中飞行时：
        *< 3.50V：触发一级低压报警（FLIGHT_STATE_LOWBAT_WARN），飞手仍可正常打杆飞回；
        *< 3.30V：触发二级严重低压强制缓降（FLIGHT_STATE_LOWBAT_LAND），自动把姿态目标回平（roll/pitch/yaw = 0），并限制最大油门平缓落地。

    2.故障锁死（Sticky Failsafe）： 一旦在空中发生 失联 > 500ms 或 大倾角/倒扣 > 60°，立刻进入 FLIGHT_STATE_FAILSAFE 切断电机。哪怕你随后走过去把飞机扶正、或者 WiFi 重新连上，电机也绝不会自己突然恢复转动——必须等你在手机上先把开关拨回“上锁（armed = false）”复位后，才能重新低油门解锁。

    3.电机输出总闸门（safety_apply_motor_gate）： 放在 mixer_quad_x 之后调用，只要处于 LOCKED 或 FAILSAFE，无条件把 m1~m4 焊死为 0.0f，彻底杜绝“油门为 0 但拿起来晃动机身时 PID 让电机误转打手”的隐患。

*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// 安全阈值常量定义（对齐 ARCHITECTURE.md §5.6）
// ============================================================================
#define SAFETY_TIMEOUT_MS         500U                           // WiFi 失联超时：500 毫秒
#define SAFETY_MAX_TILT_RAD       (60.0f * (float)M_PI / 180.0f) // 最大安全倾角：60 度（弧度制）
#define SAFETY_VBAT_WARN_V        3.50f                          // 一级低压报警 & 地面最低解锁电压：3.5V
#define SAFETY_VBAT_LAND_V        3.30f                          // 二级严重低压强制缓降电压：3.3V
#define SAFETY_ARM_MAX_THROTTLE   0.05f                          // 解锁允许的最大油门：5%（防暴冲）
#define SAFETY_LAND_MAX_THROTTLE  0.25f                          // 严重低压缓降时的油门上限：25%

// 1. 定义飞控的 5 种运行安全状态
typedef enum {
    FLIGHT_STATE_LOCKED = 0,   // 上锁待机（上电默认）：电机绝对禁止转动
    FLIGHT_STATE_ARMED,        // 正常解锁飞行
    FLIGHT_STATE_LOWBAT_WARN,  // 一级低压报警（<3.5V）：允许操控返航，状态灯快闪报警
    FLIGHT_STATE_LOWBAT_LAND,  // 二级严重低压（<3.3V）：强制回平并压低油门缓降
    FLIGHT_STATE_FAILSAFE      // 故障急停锁死（失联/翻机）：切断电机，需拨回上锁复位
} FlightState_t;

// 2. 定义安全模块的“状态盒子”
typedef struct {
    FlightState_t state;       // 当前所处的安全状态
} Safety_t;

// 3. 安全模块初始化（开机必须默认锁定）
static inline void safety_init(Safety_t *s) {
    s->state = FLIGHT_STATE_LOCKED;
}

// 4. 安全状态机核心更新函数（每拍调用）
// 输入：
//   s          —— 安全状态机指针
//   cmd        —— 遥控指令指针（若触发保护，内部会直接拦截修改油门与目标角度）
//   attitude   —— 当前 Mahony 解算出的机身姿态角（弧度制）
//   vbat       —— 当前电池电压（V）
//   cmd_age_ms —— 距离上一次收到手机遥控包过去了多少毫秒
static inline FlightState_t safety_update(Safety_t *s,
                                          ControlCommand_t *cmd,
                                          EulerAngle_t attitude,
                                          float vbat,
                                          uint32_t cmd_age_ms) {
    // 【第一优先级：飞手主动拨回“上锁”开关】
    // 只要 armed 为 false，无条件回到 LOCKED（同时也是解除 FAILSAFE 故障锁死的唯一方式）
    if (!cmd->armed) {
        s->state = FLIGHT_STATE_LOCKED;
        cmd->throttle = 0.0f;
        return s->state;
    }

    // 【第二优先级：如果已处于 FAILSAFE 故障锁死状态】
    // 在飞手把开关拨回 false 复位之前，拒绝一切动作，防止扶正飞机或重连瞬间螺旋桨暴转伤人
    if (s->state == FLIGHT_STATE_FAILSAFE) {
        cmd->throttle = 0.0f;
        cmd->roll_target = 0.0f;
        cmd->pitch_target = 0.0f;
        cmd->yaw_rate_target = 0.0f;
        return s->state;
    }

    // 检查是否触发“倾角超 60° 翻机”或“超过 500ms 失联”
    bool tilt_exceeded = (fabsf(attitude.roll) > SAFETY_MAX_TILT_RAD) ||
                         (fabsf(attitude.pitch) > SAFETY_MAX_TILT_RAD);
    bool comm_lost     = (cmd_age_ms > SAFETY_TIMEOUT_MS);

    // 【第三优先级：如果当前处于 LOCKED，检查是否满足地面安全解锁的 4 个条件】
    if (s->state == FLIGHT_STATE_LOCKED) {
        if (!comm_lost &&
            !tilt_exceeded &&
            (vbat >= SAFETY_VBAT_WARN_V) &&
            (cmd->throttle <= SAFETY_ARM_MAX_THROTTLE)) {
            // 通信正常 + 机身平放 + 地面电压 ≥ 3.5V + 油门 ≤ 5% -> 允许安全解锁！
            s->state = FLIGHT_STATE_ARMED;
        } else {
            // 任一条件不满足（如推着高油门点解锁、或地面电池已亏电），拒绝解锁！
            cmd->throttle = 0.0f;
            return s->state;
        }
    }

    // 【第四优先级：空中飞行时的实时保命检查】
    // 1) 翻机保护（>60°）或 失联保护（>500ms） -> 立即进入 FAILSAFE 急停锁死
    if (tilt_exceeded || comm_lost) {
        s->state = FLIGHT_STATE_FAILSAFE;
        cmd->throttle = 0.0f;
        cmd->roll_target = 0.0f;
        cmd->pitch_target = 0.0f;
        cmd->yaw_rate_target = 0.0f;
        return s->state;
    }

    // 2) 电池电压分级保护
    if (vbat < SAFETY_VBAT_LAND_V) {
        // 严重低压 (<3.3V)：强制机身回平，并压低最大油门缓降
        s->state = FLIGHT_STATE_LOWBAT_LAND;
        cmd->roll_target = 0.0f;
        cmd->pitch_target = 0.0f;
        cmd->yaw_rate_target = 0.0f;
        if (cmd->throttle > SAFETY_LAND_MAX_THROTTLE) {
            cmd->throttle = SAFETY_LAND_MAX_THROTTLE;
        }
    } else if (vbat < SAFETY_VBAT_WARN_V) {
        // 一级低压 (<3.5V)：点亮低压告警，但保留飞手操控权以便飞回
        s->state = FLIGHT_STATE_LOWBAT_WARN;
    } else {
        // 一切正常
        s->state = FLIGHT_STATE_ARMED;
    }

    return s->state;
}

// 5. 电机输出最终总闸门（在 mixer_quad_x 算出四路电机值之后调用！）
// 作用：只要处于 LOCKED 或 FAILSAFE，强制把 4 个电机占空比清零，彻底杜绝上锁时晃动机身导致电机误转
static inline void safety_apply_motor_gate(const Safety_t *s, MotorOutput_t *out) {
    if (s->state == FLIGHT_STATE_LOCKED || s->state == FLIGHT_STATE_FAILSAFE) {
        out->m1 = 0.0f;
        out->m2 = 0.0f;
        out->m3 = 0.0f;
        out->m4 = 0.0f;
    }
}
