#include <stdio.h>
#include "../src/flight/safety.h"
#include "../src/flight/mixer.h"

#define DEG2RAD(d) ((d) * ((float)M_PI / 180.0f))

// 辅助函数：把状态枚举转成直观的字符串名字打印出来
static const char* state_name(FlightState_t st) {
    switch (st) {
        case FLIGHT_STATE_LOCKED:      return "LOCKED     ";
        case FLIGHT_STATE_ARMED:       return "ARMED      ";
        case FLIGHT_STATE_LOWBAT_WARN: return "LOWBAT_WARN";
        case FLIGHT_STATE_LOWBAT_LAND: return "LOWBAT_LAND";
        case FLIGHT_STATE_FAILSAFE:    return "FAILSAFE   ";
        default:                       return "UNKNOWN    ";
    }
}

// 模拟一拍完整的“安全检查 -> 混控 -> 电机总闸门”并打印结果
static void run_step(const char *desc,
                     Safety_t *s,
                     ControlCommand_t cmd,
                     EulerAngle_t att,
                     float vbat,
                     uint32_t age_ms) {
    // 1. 先过安全状态机（可能会拦截修改 cmd）
    safety_update(s, &cmd, att, vbat, age_ms);

    // 2. 假设 PID 给了一点微小的横滚修正（0.10），测试上锁时电机总闸门能否拦住它
    MotorOutput_t motor;
    mixer_quad_x(cmd.throttle, 0.10f, 0.0f, 0.0f, &motor);

    // 3. 过电机最终安全总闸门
    safety_apply_motor_gate(s, &motor);

    printf("%-24s | State: %s | ThrOut: %.2f | M1=%.2f M3=%.2f\n",
           desc, state_name(s->state), cmd.throttle, motor.m1, motor.m3);
}

int main() {
    Safety_t safety;
    safety_init(&safety);

    EulerAngle_t flat_att = { 0.0f, 0.0f, 0.0f };
    EulerAngle_t flip_att = { DEG2RAD(75.0f), 0.0f, 0.0f }; // 横滚 75°（超 60° 红线）

    printf("============================= Safety State Machine Test =============================\n");

    // 场景 1：危险操作！油门在 50% 时直接点解锁 -> 必须拒绝解锁，且电机被总闸门焊死为 0
    ControlCommand_t c1 = { 0.50f, 0.0f, 0.0f, 0.0f, true };
    run_step("1. Arm w/ High Throttle", &safety, c1, flat_att, 3.90f, 20);

    // 场景 2：油门拉回 0% 正常解锁
    ControlCommand_t c2 = { 0.00f, 0.0f, 0.0f, 0.0f, true };
    run_step("2. Arm w/ Zero Throttle", &safety, c2, flat_att, 3.90f, 20);

    // 场景 3：解锁成功后，推油门到 60% 正常飞行
    ControlCommand_t c3 = { 0.60f, 0.2f, 0.0f, 0.0f, true };
    run_step("3. Normal Flight (3.9V)", &safety, c3, flat_att, 3.90f, 20);

    // 场景 4：电池降到 3.40V（<3.5V） -> 触发一级低压报警，但允许保持 60% 油门返航
    run_step("4. LowBat Warn (3.40V)",  &safety, c3, flat_att, 3.40f, 20);

    // 场景 5：电池降到 3.20V（<3.3V） -> 触发二级严重低压，油门被强行压到 0.25 缓降
    run_step("5. LowBat Land (3.20V)",  &safety, c3, flat_att, 3.20f, 20);

    // 场景 6：WiFi 断连达到 600ms（>500ms） -> 触发 FAILSAFE 急停，电机全部归零
    run_step("6. WiFi Lost (600ms)",    &safety, c3, flat_att, 3.80f, 600);

    // 场景 7：WiFi 信号恢复（10ms），但飞手还没拨回上锁开关 -> 必须继续锁死在 FAILSAFE！
    run_step("7. WiFi Back (No Reset)", &safety, c3, flat_att, 3.80f, 10);

    // 场景 8：飞手拨回 armed=false 完成复位 -> 回到 LOCKED
    ControlCommand_t c_disarm = { 0.00f, 0.0f, 0.0f, 0.0f, false };
    run_step("8. Disarm to Reset",      &safety, c_disarm, flat_att, 3.80f, 10);

    // 场景 9：重新低油门解锁，随后机身歪到 75°（>60°） -> 立即触发翻机急停 FAILSAFE！
    run_step("9a. Re-Arm (Zero Thr)",   &safety, c2, flat_att, 3.80f, 10);
    run_step("9b. Crash Tilt (75 deg)", &safety, c3, flip_att, 3.80f, 10);

    printf("=====================================================================================\n");
    return 0;
}