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

    // 场景 5c：严重低压迫降叠加 75° 阵风倾角 -> 仍保持 LOWBAT_LAND 单向锁存且油门仍被严限 0.25f
    ControlCommand_t c5c = { 0.60f, 0.2f, 0.0f, 0.0f, true };
    run_step("5c. LowBat + 75 deg Gust", &safety, &c5c, &motor, flip_att, 3.60f, 20);
    CHECK(safety.state == FLIGHT_STATE_LOWBAT_LAND && fabsf(c5c.throttle - 0.25f) < 1e-5f,
          "lowbat land latch & 0.25 throttle cap preserved during gust recovery");

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

    // 场景 9（E-02 升级）：大倾角分级保护、滞回防抖、怠速翻倒与脏数据防火墙
    // 9a. 水平正常解锁
    ControlCommand_t c9a = { 0.00f, 0.0f, 0.0f, 0.0f, true };
    run_step("9a. Re-Arm (Zero Thr)", &safety, &c9a, &motor, flat_att, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_ARMED, "9a. re-arm flat succeeds");

    // 9b. 阵风吹偏至 75° 持续 80 拍（200ms < 0.8s）：归零目标并限油门 45% 自稳救机；
    //     回到 52°（仍 >45° 滞回角）时保持救机且不清空计时（防撞墙反复反弹清零），回到 30°（<=45°）才彻底复位
    ControlCommand_t c9b = { 0.65f, 0.30f, -0.20f, 0.80f, true };
    for (int i = 0; i < 80; ++i) {
        c9b = (ControlCommand_t){ 0.65f, 0.30f, -0.20f, 0.80f, true };
        safety_update(&safety, &c9b, flip_att, 3.80f, 10);
    }
    mixer_quad_x(c9b.throttle, 0.10f, 0.0f, 0.0f, &motor);
    safety_apply_motor_gate(&safety, &motor);
    printf("%-24s | State: %s | ThrOut: %.2f | M1=%.2f M3=%.2f\n",
           "9b-1. Gust 75deg(200ms)", state_name(safety.state), c9b.throttle, motor.m1, motor.m3);
    CHECK(safety.state == FLIGHT_STATE_ARMED &&
          safety.high_tilt_ticks == 80U &&
          fabsf(c9b.throttle - SAFETY_TILT_RECOVER_MAX_THROTTLE) < 1e-5f &&
          c9b.roll_target == 0.0f && c9b.pitch_target == 0.0f && c9b.yaw_rate_target == 0.0f &&
          motor.m1 > 0.0f && motor.m3 > 0.0f,
          "9b-1. 75 deg gust (<0.8s) keeps motors alive, zeroes targets & caps throttle to 0.45");

    EulerAngle_t mid_52 = { DEG2RAD(52.0f), 0.0f, 0.0f };
    ControlCommand_t c9b_hyst = { 0.65f, 0.10f, 0.0f, 0.0f, true };
    run_step("9b-2. Hyst Hold (52deg)", &safety, &c9b_hyst, &motor, mid_52, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_ARMED &&
          safety.high_tilt_ticks == 81U &&
          c9b_hyst.roll_target == 0.0f &&
          fabsf(c9b_hyst.throttle - SAFETY_TILT_RECOVER_MAX_THROTTLE) < 1e-5f,
          "9b-2. hysteresis prevents bounce at 52 deg (>45 deg) from clearing tilt counter");

    EulerAngle_t ok_30 = { DEG2RAD(30.0f), 0.0f, 0.0f };
    ControlCommand_t c9b_rec = { 0.65f, 0.10f, 0.0f, 0.0f, true };
    run_step("9b-3. Recovered (30deg)", &safety, &c9b_rec, &motor, ok_30, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_ARMED &&
          safety.high_tilt_ticks == 0U &&
          fabsf(c9b_rec.throttle - 0.65f) < 1e-5f,
          "9b-3. recovering <= 45 deg resets high_tilt_ticks and restores pilot throttle");

    // 9c. 持续不可恢复倾斜（65° 维持 >= 0.8s / 320 拍）：第 319 拍仍在救机，第 320 拍触发 FAILSAFE 断电
    EulerAngle_t tilt_65 = { DEG2RAD(65.0f), 0.0f, 0.0f };
    ControlCommand_t c9c = { 0.60f, 0.0f, 0.0f, 0.0f, true };
    for (int i = 1; i <= 319; ++i) {
        c9c = (ControlCommand_t){ 0.60f, 0.0f, 0.0f, 0.0f, true };
        safety_update(&safety, &c9c, tilt_65, 3.80f, 10);
    }
    mixer_quad_x(c9c.throttle, 0.10f, 0.0f, 0.0f, &motor);
    safety_apply_motor_gate(&safety, &motor);
    CHECK(safety.state == FLIGHT_STATE_ARMED &&
          safety.high_tilt_ticks == 319U &&
          motor.m1 > 0.0f && motor.m3 > 0.0f,
          "9c-1. 65 deg tilt at tick 319 (797.5ms) still attempts recovery");

    c9c = (ControlCommand_t){ 0.60f, 0.0f, 0.0f, 0.0f, true };
    run_step("9c-2. Timeout (0.8s)", &safety, &c9c, &motor, tilt_65, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE &&
          safety.high_tilt_ticks == 0U &&
          motor.m1 == 0.0f && motor.m3 == 0.0f,
          "9c-2. sustained >60 deg for 320 ticks (0.8s) triggers FAILSAFE & cuts motors");

    // 9d. 对角复合倾角（48°+48°≈63.4°）与真倒扣（>110° 含 Mahony 万向锁表示 {180°, 55°}）检测
    ControlCommand_t c9_disarm = { 0.00f, 0.0f, 0.0f, 0.0f, false };
    ControlCommand_t c9_rearm  = { 0.00f, 0.0f, 0.0f, 0.0f, true };
    safety_update(&safety, &c9_disarm, flat_att, 3.80f, 10);
    safety_update(&safety, &c9_rearm,  flat_att, 3.80f, 10);

    EulerAngle_t diag_tilt = { DEG2RAD(48.0f), DEG2RAD(48.0f), 0.0f };
    ControlCommand_t c9d1 = { 0.60f, 0.1f, 0.1f, 0.0f, true };
    run_step("9d-1. Diag 48+48", &safety, &c9d1, &motor, diag_tilt, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_ARMED &&
          safety.high_tilt_ticks == 1U &&
          fabsf(c9d1.throttle - SAFETY_TILT_RECOVER_MAX_THROTTLE) < 1e-5f,
          "9d-1. combined 48+48 deg diagonal tilt (63.4 deg) detected via 3D cosine");

    // Mahony 俯仰 125° 倒扣对应的欧拉角折返形式：roll=180°, pitch=55°
    EulerAngle_t invert_125 = { DEG2RAD(180.0f), DEG2RAD(55.0f), 0.0f };
    ControlCommand_t c9d2 = { 0.60f, 0.0f, 0.0f, 0.0f, true };
    run_step("9d-2. Invert 125 deg", &safety, &c9d2, &motor, invert_125, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE &&
          safety.high_tilt_ticks == 0U &&
          motor.m1 == 0.0f && motor.m3 == 0.0f,
          "9d-2. >110 deg extreme inversion (Mahony {180,55}) cuts motors in 1 tick");

    // 9e. 歪斜地面禁止解锁（且不泄漏计时）+ 非法 NaN 姿态单拍直接急停
    safety_update(&safety, &c9_disarm, flat_att, 3.80f, 10);
    c9_rearm = (ControlCommand_t){ 0.00f, 0.0f, 0.0f, 0.0f, true };
    run_step("9e-1. Arm Tilted(65deg)", &safety, &c9_rearm, &motor, tilt_65, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_LOCKED &&
          safety.high_tilt_ticks == 0U &&
          motor.m1 == 0.0f && motor.m3 == 0.0f,
          "9e-1. refuses ground arming when tilted >60 deg without ticking timer");

    c9_rearm = (ControlCommand_t){ 0.00f, 0.0f, 0.0f, 0.0f, true };
    safety_update(&safety, &c9_rearm, flat_att, 3.80f, 10);
    EulerAngle_t nan_att = { NAN, 0.0f, 0.0f };
    ControlCommand_t c9e2 = { 0.50f, 0.0f, 0.0f, 0.0f, true };
    run_step("9e-2. NaN Attitude", &safety, &c9e2, &motor, nan_att, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE &&
          motor.m1 == 0.0f && motor.m3 == 0.0f,
          "9e-2. NaN attitude immediately triggers FAILSAFE and cuts motors");

    // 9f. 地面怠速翻倒单拍断桨保护 + 非法 NaN 遥控指令单拍急停
    safety_update(&safety, &c9_disarm, flat_att, 3.80f, 10);
    c9_rearm = (ControlCommand_t){ 0.00f, 0.0f, 0.0f, 0.0f, true };
    safety_update(&safety, &c9_rearm, flat_att, 3.80f, 10);
    ControlCommand_t c9f1 = { 0.02f, 0.0f, 0.0f, 0.0f, true };
    run_step("9f-1. Idle Tip (75deg)", &safety, &c9f1, &motor, flip_att, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE &&
          safety.high_tilt_ticks == 0U &&
          motor.m1 == 0.0f && motor.m3 == 0.0f,
          "9f-1. tilted >60 deg during ground-idle (<=5% thr) cuts motors in 1 tick");

    safety_update(&safety, &c9_disarm, flat_att, 3.80f, 10);
    c9_rearm = (ControlCommand_t){ 0.00f, 0.0f, 0.0f, 0.0f, true };
    safety_update(&safety, &c9_rearm, flat_att, 3.80f, 10);
    ControlCommand_t c9f2 = { NAN, 0.0f, 0.0f, 0.0f, true };
    run_step("9f-2. NaN Command", &safety, &c9f2, &motor, flat_att, 3.80f, 10);
    CHECK(safety.state == FLIGHT_STATE_FAILSAFE &&
          motor.m1 == 0.0f && motor.m3 == 0.0f,
          "9f-2. NaN control command triggers immediate FAILSAFE and cuts motors");

    printf("=====================================================================================\n");
    if (g_failures == 0) {
        printf("ALL SAFETY TESTS PASSED!\n");
        return 0;
    }
    printf("SAFETY TESTS FAILED: %d\n", g_failures);
    return 1;
}
