#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// 安全阈值常量定义（对齐 ARCHITECTURE.md §5.6，支持编译期覆盖）
// ============================================================================
#ifndef SAFETY_TIMEOUT_MS
#define SAFETY_TIMEOUT_MS                500U                                // 遥控失联超时：500 毫秒
#endif
#define SAFETY_MAX_TILT_RAD              (60.0f  * ((float)M_PI / 180.0f))   // 一级大倾角警戒角：60 度（触发回平救机并开始计时）
#define SAFETY_TILT_RECOVER_OK_RAD       (45.0f  * ((float)M_PI / 180.0f))   // 救机退出滞回角：45 度（防止撞墙/阵风在 60° 边界反弹清零计时器）
#define SAFETY_EXTREME_TILT_RAD          (110.0f * ((float)M_PI / 180.0f))   // 二级极端翻机角：110 度（彻底倒扣，单拍急停锁死）
#define SAFETY_TILT_GRACE_TICKS          320U                                // 救机宽限期：320 拍（基于 400Hz 控制环路 = 0.8 秒）
#define SAFETY_TILT_RECOVER_MAX_THROTTLE 0.45f                               // 救机油门上限：45%（映射后 mix=0.40 匹配内环力矩上限且≈悬停推力，防掉高与横冲）
#define SAFETY_VBAT_WARN_V               3.50f                               // 一级低压报警 & 地面最低解锁电压：3.5V
#define SAFETY_VBAT_LAND_V               3.30f                               // 二级严重低压强制缓降电压：3.3V
#define SAFETY_ARM_MAX_THROTTLE          0.05f                               // 解锁允许的最大油门：5%（防暴冲）
#define SAFETY_LAND_MAX_THROTTLE         0.25f                               // 严重低压缓降时的油门上限：25%

// 1. 定义飞控的 5 种运行安全状态
typedef enum {
    FLIGHT_STATE_LOCKED = 0,   // 上锁待机（上电默认）：电机绝对禁止转动
    FLIGHT_STATE_ARMED,        // 正常解锁飞行
    FLIGHT_STATE_LOWBAT_WARN,  // 一级低压报警（<3.5V）：允许操控返航，状态灯快闪报警
    FLIGHT_STATE_LOWBAT_LAND,  // 二级严重低压（<3.3V，单向锁存）：强制回平并压低油门缓降
    FLIGHT_STATE_FAILSAFE      // 故障急停锁死（失联/翻机，单向锁存）：切断电机，需拨回上锁复位
} FlightState_t;

// 2. 定义安全模块的“状态盒子”
typedef struct {
    FlightState_t state;             // 当前所处的安全状态
    uint32_t      high_tilt_ticks;   // 处于大倾角救机滞回区间的连续节拍数（>0 即表示仍在救机滞回中）
} Safety_t;

// 内部辅助函数：将三轴目标姿态/角速度全部归零
static inline void safety_zero_attitude_targets(ControlCommand_t *cmd) {
    cmd->roll_target = 0.0f;
    cmd->pitch_target = 0.0f;
    cmd->yaw_rate_target = 0.0f;
}

// 3. 安全模块初始化（开机必须默认锁定）
static inline void safety_init(Safety_t *s) {
    s->state           = FLIGHT_STATE_LOCKED;
    s->high_tilt_ticks = 0U;
}

// 4. 安全状态机核心更新函数（每拍调用）
//   - cmd        : 遥控指令（throttle [0.0, 1.0], roll/pitch_target [rad], yaw_rate_target [rad/s]）
//   - attitude   : 当前机体欧拉角（FRD [rad]）
//   - vbat       : 滤波后电池电压 [V]
//   - cmd_age_ms : 最近一帧有效遥控包的包龄 [ms]
static inline FlightState_t safety_update(Safety_t *s,
                                          ControlCommand_t *cmd,
                                          EulerAngle_t attitude,
                                          float vbat,
                                          uint32_t cmd_age_ms) {
    // 【第一优先级：飞手主动拨回“上锁”开关】
    // 只要 armed 为 false，无条件回到 LOCKED（这也是解除 FAILSAFE 和 LOWBAT_LAND 锁存的唯一方式）
    if (!cmd->armed) {
        s->state           = FLIGHT_STATE_LOCKED;
        s->high_tilt_ticks = 0U;
        cmd->throttle      = 0.0f;
        safety_zero_attitude_targets(cmd);
        return s->state;
    }

    // 【第二优先级：如果已处于 FAILSAFE 故障锁死状态】
    if (s->state == FLIGHT_STATE_FAILSAFE) {
        s->high_tilt_ticks = 0U;
        cmd->throttle      = 0.0f;
        safety_zero_attitude_targets(cmd);
        return s->state;
    }

    // 校验遥控指令有限性并钳位油门到 [0.0, 1.0]（防 NaN / Inf / 负值穿透到混控器与 PWM）
    bool cmd_valid = isfinite(cmd->throttle) &&
                     isfinite(cmd->roll_target) &&
                     isfinite(cmd->pitch_target) &&
                     isfinite(cmd->yaw_rate_target);
    if (cmd_valid) {
        if (cmd->throttle < 0.0f) {
            cmd->throttle = 0.0f;
        } else if (cmd->throttle > 1.0f) {
            cmd->throttle = 1.0f;
        }
    }

    // 检查是否触发倾角警戒/滞回退出/倒扣或超过 500ms 失联：
    // 采用三维重力投影 cos(roll)*cos(pitch) 计算真实空间倾角，消除 pitch 在 ±90° 万向锁折返导致的 >110° 漏检，并防 NaN 绕过
    bool  attitude_valid = isfinite(attitude.roll) && isfinite(attitude.pitch);
    float cos_tilt       = attitude_valid ? (cosf(attitude.roll) * cosf(attitude.pitch)) : -1.0f;
    bool  tilt_enter     = !attitude_valid || (cos_tilt <  cosf(SAFETY_MAX_TILT_RAD));
    bool  tilt_clear     =  attitude_valid && (cos_tilt >= cosf(SAFETY_TILT_RECOVER_OK_RAD));
    bool  tilt_extreme   = !attitude_valid || (cos_tilt <  cosf(SAFETY_EXTREME_TILT_RAD));
    bool  comm_lost      = (cmd_age_ms > SAFETY_TIMEOUT_MS);

    // 【第三优先级：如果当前处于 LOCKED，检查是否满足地面安全解锁的 4 个条件】
    if (s->state == FLIGHT_STATE_LOCKED) {
        if (cmd_valid &&
            !comm_lost &&
            !tilt_enter &&
            isfinite(vbat) &&
            (vbat >= SAFETY_VBAT_WARN_V) &&
            (cmd->throttle <= SAFETY_ARM_MAX_THROTTLE)) {
            // 通信正常 + 机身平放(<=60°) + 地面电压 ≥ 3.5V + 油门 ≤ 5% -> 允许安全解锁！
            s->state           = FLIGHT_STATE_ARMED;
            s->high_tilt_ticks = 0U;
        } else {
            // 任一条件不满足，拒绝解锁并清空全部控制目标（且禁止在锁定态累计倾角计时）
            s->high_tilt_ticks = 0U;
            cmd->throttle      = 0.0f;
            safety_zero_attitude_targets(cmd);
            return s->state;
        }
    }

    // 【第四优先级：空中飞行时的实时保命检查】
    // 1) 非法遥控数值 / 失联超时（>500ms） / 极端倒扣与非法姿态（>110°） -> 立即进入 FAILSAFE 急停锁死
    if (!cmd_valid || comm_lost || tilt_extreme) {
        s->state           = FLIGHT_STATE_FAILSAFE;
        s->high_tilt_ticks = 0U;
        cmd->throttle      = 0.0f;
        safety_zero_attitude_targets(cmd);
        return s->state;
    }

    // 2) 中度大倾角分级救机保护（>60° 触发，回正至 <=45° 滞回角才退出）：
    //    - 若油门处于地面怠速死区（<=5%），controller_step 已关闭姿态 PID 无法自稳救机，单拍直切 FAILSAFE 断桨保护；
    //    - 若处于飞行油门（>5%），归零摇杆目标并压低油门上限到 45% 全力自稳救回；连续 0.8s (320 拍) 未回到 <=45° 则急停锁死
    if (tilt_enter || (s->high_tilt_ticks > 0U && !tilt_clear)) {
        if (cmd->throttle <= SAFETY_ARM_MAX_THROTTLE) {
            s->state           = FLIGHT_STATE_FAILSAFE;
            s->high_tilt_ticks = 0U;
            cmd->throttle      = 0.0f;
            safety_zero_attitude_targets(cmd);
            return s->state;
        }
        s->high_tilt_ticks++;
        if (s->high_tilt_ticks >= SAFETY_TILT_GRACE_TICKS) {
            s->state           = FLIGHT_STATE_FAILSAFE;
            s->high_tilt_ticks = 0U;
            cmd->throttle      = 0.0f;
            safety_zero_attitude_targets(cmd);
            return s->state;
        }
        safety_zero_attitude_targets(cmd);
        if (cmd->throttle > SAFETY_TILT_RECOVER_MAX_THROTTLE) {
            cmd->throttle = SAFETY_TILT_RECOVER_MAX_THROTTLE;
        }
    } else {
        s->high_tilt_ticks = 0U;
    }

    // 3) 电池电压分级保护（注意：LOWBAT_LAND 一旦触发即单向锁存，防减油门后电压回弹抽搐！）
    if (s->state == FLIGHT_STATE_LOWBAT_LAND || (isfinite(vbat) && vbat < SAFETY_VBAT_LAND_V)) {
        s->state = FLIGHT_STATE_LOWBAT_LAND;
        safety_zero_attitude_targets(cmd);
        if (cmd->throttle > SAFETY_LAND_MAX_THROTTLE) {
            cmd->throttle = SAFETY_LAND_MAX_THROTTLE;
        }
    } else if (isfinite(vbat) && vbat < SAFETY_VBAT_WARN_V) {
        // 一级低压 (<3.5V)：点亮低压告警，但保留飞手操控权以便飞回
        s->state = FLIGHT_STATE_LOWBAT_WARN;
    } else {
        // 一切正常（或处于 0.8s 倾角救机宽限窗口内持续自稳中）
        s->state = FLIGHT_STATE_ARMED;
    }

    return s->state;
}

// 5. 电机输出最终总闸门（在 mixer_quad_x 算出四路电机值之后调用！兼做 NaN 硬件写入防火墙）
static inline void safety_apply_motor_gate(const Safety_t *s, MotorOutput_t *out) {
    if (s->state == FLIGHT_STATE_LOCKED ||
        s->state == FLIGHT_STATE_FAILSAFE ||
        !isfinite(out->m1) || !isfinite(out->m2) ||
        !isfinite(out->m3) || !isfinite(out->m4)) {
        out->m1 = 0.0f;
        out->m2 = 0.0f;
        out->m3 = 0.0f;
        out->m4 = 0.0f;
    }
}
