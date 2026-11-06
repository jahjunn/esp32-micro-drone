#include <stdio.h>
#include <math.h>
#include "../src/flight/safety.h"
#include "../src/flight/mixer.h"

#define DEG2RAD(d) ((d) * ((float)M_PI / 180.0f))

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] %s (line %d)\n", msg, __LINE__); \
        g_failures++; \
    } else { \
        printf("[PASS] %s\n", msg); \
    } \
} while (0)

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

static void run_step(const char *desc,
                     Safety_t *s,
                     ControlCommand_t *cmd,
                     MotorOutput_t *motor,
                     EulerAngle_t att,
                     float vbat,
                     uint32_t age_ms) {
    safety_update(s, cmd, att, vbat, age_ms);
    mixer_quad_x(cmd->throttle, 0.10f, 0.0f, 0.0f, motor);
    safety_apply_motor_gate(s, motor);

    printf("%-24s | State: %s | ThrOut: %.2f | M1=%.2f M3=%.2f\n",
           desc, state_name(s->state), cmd->throttle, motor->m1, motor->m3);
}

int main(void) {
    Safety_t safety;
    safety_init(&safety);

    EulerAngle_t flat_att = { 0.0f, 0.0f, 0.0f };
    EulerAngle_t flip_att = { DEG2RAD(75.0f), 0.0f, 0.0f };
    MotorOutput_t motor;

    printf("============================= Safety State Machine Test =============================\n");

    // 场景 1：油门 50% 时点解锁 -> 拒绝解锁，电机焊死为 0
    ControlCommand_t c1 = { 0.50f, 0.2f, 0.1f, 0.0f, true };
    run_step("1. Arm w/ High Throttle", &safety, &c1, &motor, flat_att, 3.90f, 20);
    CHECK(safety.state == FLIGHT_STATE_LOCKED && motor.m1 == 0.0f && motor.m3 == 0.0f,
          "high-throttle arming rejected and motors gated to 0");

    // 场景 2：油门 0% 正常解锁
    ControlCommand_t c2 = { 0.00f, 0.0f, 0.0f, 0.0f, true };
    run_step("2. Arm w/ Zero Throttle", &safety, &c2, &motor, flat_att, 3.90f, 20);
    CHECK(safety.state == FLIGHT_STATE_ARMED, "zero-throttle arming succeeds");

    // 场景 3：60% 油门正常飞行
    ControlCommand_t c3 = { 0.60f, 0.2f, 0.0f, 0.0f, true };
    run_step("3. Normal Flight (3.9V)", &safety, &c3, &motor, flat_att, 3.90f, 20);
    CHECK(safety.state == FLIGHT_STATE_ARMED && fabsf(c3.throttle - 0.60f) < 1e-5f,
          "normal flight preserves 60% throttle");

    // 场景 4：3.40V 一级低压告警
    ControlCommand_t c4 = { 0.60f, 0.2f, 0.0f, 0.0f, true };
    run_step("4. LowBat Warn (3.40V)", &safety, &c4, &motor, flat_att, 3.40f, 20);
    CHECK(safety.state == FLIGHT_STATE_LOWBAT_WARN && fabsf(c4.throttle - 0.60f) < 1e-5f,
          "lowbat warn alerts while preserving pilot control");

    // 场景 5a：3.20V 二级严重低压 -> 强制回平 + 油门压到 0.25
    ControlCommand_t c5a = { 0.60f, 0.2f, -0.1f, 0.5f, true };
    run_step("5a. LowBat Land (3.20V)", &safety, &c5a, &motor, flat_att, 3.20f, 20);
    CHECK(safety.state == FLIGHT_STATE_LOWBAT_LAND &&
          fabsf(c5a.throttle - 0.25f) < 1e-5f &&
          c5a.roll_target == 0.0f && c5a.pitch_target == 0.0f,
          "lowbat land caps throttle to 0.25 and levels attitude");

    // 场景 5b：【关键回归测试】减油门后电池电压回弹到 3.45V 甚至 3.60V -> 必须保持 LOWBAT_LAND 锁存！
    ControlCommand_t c5b = { 0.60f, 0.2f, 0.0f, 0.0f, true };
    run_step("5b. Vbat Rebound (3.60V)", &safety, &c5b, &motor, flat_att, 3.60f, 20);
    CHECK(safety.state == FLIGHT_STATE_LOWBAT_LAND && fabsf(c5b.throttle - 0.25f) < 1e-5f,
          "lowbat land stays latched on voltage rebound (no oscillation)");

    // 场景 6：失联 600ms (>500ms) -> FAILSAFE 急停
    ControlCommand_t c6 = { 0.60f, 0.2f, 0.0f, 0.0f, true };
    run_step("6. Comm Lost (600ms)", &safety, &c6, &motor, flat_att, 3.80f, 600);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE && motor.m1 == 0.0f && motor.m3 == 0.0f,
          "comm timeout triggers FAILSAFE and cuts motors");

    // 场景 7：通信恢复但未拨回上锁 -> 继续锁死在 FAILSAFE
    ControlCommand_t c7 = { 0.60f, 0.2f, 0.0f, 0.0f, true };
    run_step("7. Comm Back (No Reset)", &safety, &c7, &motor, flat_att, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE && motor.m1 == 0.0f,
          "FAILSAFE remains sticky until disarmed");

    // 场景 8：拨回 armed=false 完成复位 -> 回到 LOCKED
    ControlCommand_t c8 = { 0.00f, 0.0f, 0.0f, 0.0f, false };
    run_step("8. Disarm to Reset", &safety, &c8, &motor, flat_att, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_LOCKED, "disarm resets state machine to LOCKED");

    // 场景 9：重新解锁后机身倾角 75° (>60°) -> 立即触发 FAILSAFE
    ControlCommand_t c9a = { 0.00f, 0.0f, 0.0f, 0.0f, true };
    ControlCommand_t c9b = { 0.60f, 0.0f, 0.0f, 0.0f, true };
    run_step("9a. Re-Arm (Zero Thr)", &safety, &c9a, &motor, flat_att, 3.80f, 10);
    run_step("9b. Crash Tilt (75 deg)", &safety, &c9b, &motor, flip_att, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE && motor.m1 == 0.0f && motor.m3 == 0.0f,
          "tilt > 60 deg triggers FAILSAFE immediately");

    printf("=====================================================================================\n");
    if (g_failures == 0) {
        printf("ALL SAFETY TESTS PASSED!\n");
        return 0;
    }
    printf("SAFETY TESTS FAILED: %d\n", g_failures);
    return 1;
}