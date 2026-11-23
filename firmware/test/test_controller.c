#include <stdio.h>
#include <math.h>
#include "../src/flight/controller.h"

#define DEG2RAD(d) ((d) * ((float)M_PI / 180.0f))
#define RAD2DEG(r) ((r) * (180.0f / (float)M_PI))

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] %s (line %d)\n", msg, __LINE__); \
        g_failures++; \
    } else { \
        printf("[PASS] %s\n", msg); \
    } \
} while (0)

int main(void) {
    FlightController_t fc;
    controller_init(&fc);

    MotorOutput_t motors = {0};
    Vector3_t zero_gyro  = {0.0f, 0.0f, 0.0f};
    Vector3_t flat_accel = {0.0f, 0.0f, 1.0f};

    Vector3_t mapped_gyro, gravity_observation;
    controller_map_flu_to_frd((Vector3_t){1.0f, 2.0f, 3.0f},
                             (Vector3_t){0.2f, 0.3f, 0.9f}, &mapped_gyro, &gravity_observation);
    CHECK(mapped_gyro.x == 1.0f && mapped_gyro.y == -2.0f && mapped_gyro.z == -3.0f,
          "FLU gyro rotates to FRD with signs +,-,-");
    CHECK(fabsf(gravity_observation.x + 0.2f) < 1e-6f &&
          fabsf(gravity_observation.y - 0.3f) < 1e-6f &&
          fabsf(gravity_observation.z - 0.9f) < 1e-6f,
          "Accelerometer mapping returns FRD gravity observation -f, not raw specific force");

    printf("================ Cascaded Flight Controller Test ================\n");

    // 1. 测试陀螺仪未完成校准前禁止解锁 + 手抖自动重测 + 200 拍静止校准通过
    ControlCommand_t cmd_arm = {0.0f, 0.0f, 0.0f, 0.0f, true};
    Vector3_t biased_gyro = {DEG2RAD(2.5f), DEG2RAD(-1.5f), DEG2RAD(0.8f)};

    // 先采 50 拍正常数据，再注入 0.18 rad/s 峰峰值，验证工程规定的 0.15 阈值。
    for (int i = 0; i < 50; ++i) {
        controller_step(&fc, biased_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    }
    Vector3_t shake_gyro = {biased_gyro.x + 0.18f, biased_gyro.y, biased_gyro.z};
    FlightState_t st = controller_step(&fc, shake_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    CHECK(!fc.gyro_calibrated && fc.cal_count == 0U && st == FLIGHT_STATE_LOCKED,
          "1a. Hand shake during gyro calibration resets sample counter & blocks arming");

    // 随后模拟峰峰值 0.10 rad/s 的静止噪声，验证可完成校准并准确算出零偏。
    CHECK(controller_is_calibrating(&fc),
          "1b. controller_is_calibrating() reports true before calibration completes");
    for (int i = 0; i < (int)CTRL_GYRO_CAL_SAMPLES; ++i) {
        float micro_vib = (i % 2 == 0) ? 0.05f : -0.05f;
        Vector3_t desk_gyro = {biased_gyro.x + micro_vib, biased_gyro.y, biased_gyro.z};
        st = controller_step(&fc, desk_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    }
    CHECK(!controller_is_calibrating(&fc) &&
          fabsf(fc.gyro_bias.x - biased_gyro.x) < 1e-5f &&
          fabsf(fc.gyro_bias.y - biased_gyro.y) < 1e-5f,
          "1c. 200 ticks with 0.10 rad/s noise complete calibration accurately");

    // 2. 测试地面怠速静默机制：刚解锁且油门为 0 时，哪怕桌面歪了 5 度，四路电机也严格保持 0.12 不乱加速
    Vector3_t table_tilt_accel = {0.0f, sinf(DEG2RAD(5.0f)), cosf(DEG2RAD(5.0f))};
    st = controller_step(&fc, biased_gyro, table_tilt_accel, cmd_arm, 3.90f, 10U, &motors);
    printf("Ground Idle (5 deg uneven table) | M1=%.2f M2=%.2f M3=%.2f M4=%.2f\n",
           motors.m1, motors.m2, motors.m3, motors.m4);
    CHECK(st == FLIGHT_STATE_ARMED &&
          fabsf(motors.m1 - MOTOR_IDLE_DEFAULT) < 1e-5f &&
          fabsf(motors.m3 - MOTOR_IDLE_DEFAULT) < 1e-5f,
          "2. Ground-idle keeps uniform 12% motor spin on uneven table without fighting");

    // 3. 测试电池电压 IIR 滤波抗毛刺：正常 3.85V 飞行中混入 1 拍 3.10V 尖峰，绝不能误锁存 LOWBAT_LAND
    ControlCommand_t cmd_hover = {0.50f, 0.0f, 0.0f, 0.0f, true};
    st = controller_step(&fc, biased_gyro, flat_accel, cmd_hover, 3.10f, 10U, &motors);
    CHECK(st == FLIGHT_STATE_ARMED && fc.vbat_filtered > 3.70f,
          "3. Battery LPF rejects single-tick 3.10V PWM glitch without false LOWBAT_LAND");

    // 4. 测试 MPU6050 芯片 FLU 坐标系到飞控 FRD 坐标系映射 + 400Hz/100Hz 串级自稳闭环
    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);

    float sim_roll = 0.0f;              // 从 0 度水平悬停开始
    float sim_rate = DEG2RAD(180.0f);   // 突遇强阵风冲击：产生 +180 deg/s 的初始向右滚转角速度
    bool right_motors_boosted = false;

    for (int step = 1; step <= 400; ++step) {
        // 模拟正面朝上贴装的 MPU6050 (FLU) 原始读数
        Vector3_t chip_gyro_flu  = {sim_rate, 0.0f, 0.0f};
        Vector3_t chip_accel_flu = {0.0f, sinf(sim_roll), cosf(sim_roll)};
        Vector3_t gyro_frd, accel_frd;
        controller_map_flu_to_frd(chip_gyro_flu, chip_accel_flu, &gyro_frd, &accel_frd);

        controller_step(&fc, gyro_frd, accel_frd, cmd_hover, 3.90f, 10U, &motors);

        if (step == 10) {
            printf("Step 10 (Roll=+%.1f deg) | M1(R)=%.2f M2(R)=%.2f M3(L)=%.2f M4(L)=%.2f\n",
                   RAD2DEG(sim_roll), motors.m1, motors.m2, motors.m3, motors.m4);
            if (motors.m1 > motors.m3 && motors.m2 > motors.m4) {
                right_motors_boosted = true;
            }
        }

        float net_roll_moment = ((motors.m3 + motors.m4) - (motors.m1 + motors.m2)) * 25.0f;
        sim_rate += (net_roll_moment - 2.0f * sim_rate) * CTRL_DT;
        sim_roll += sim_rate * CTRL_DT;

        if (step % 100 == 0) {
            printf("Time %.2fs | True Roll: %6.2f deg | Est Roll: %6.2f deg\n",
                   step * CTRL_DT, RAD2DEG(sim_roll), RAD2DEG(fc.attitude.roll));
        }
    }

    CHECK(right_motors_boosted,
          "4a. FLU->FRD mapping + cascade PID boosts right motors (M1,M2 > M3,M4) on right tilt");
    CHECK(fabsf(RAD2DEG(sim_roll)) < 1.0f && fabsf(RAD2DEG(fc.attitude.roll)) < 1.0f,
          "4b. 400Hz/100Hz cascaded loop stabilizes +15 deg disturbance to < 1.0 deg in 1s");

    // 5. 全通道黑盒连通性探测（E-01）：验证 Pitch 抬头与 Yaw 右转差动符号严格符合 FRD 契约
    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);

    ControlCommand_t cmd_pitch_up = {0.50f, 0.0f, DEG2RAD(10.0f), 0.0f, true};
    controller_step(&fc, zero_gyro, flat_accel, cmd_pitch_up, 3.90f, 10U, &motors);
    CHECK(motors.m1 > motors.m2 + 0.05f && motors.m4 > motors.m3 + 0.05f,
          "5a. Pitch +10 deg nose-up command boosts front motors (M1,M4 > M2,M3)");

    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    ControlCommand_t cmd_yaw_cw = {0.50f, 0.0f, 0.0f, 1.0f, true};
    controller_step(&fc, zero_gyro, flat_accel, cmd_yaw_cw, 3.90f, 10U, &motors);
    CHECK(motors.m1 > motors.m2 + 0.05f && motors.m3 > motors.m4 + 0.05f,
          "5b. Yaw +1.0 rad/s CW command boosts CCW motors (M1,M3 > M2,M4)");

    // 6. 怠速->飞行分界点连续性扫描（E-05）：油门从 0.00 按 0.005 步进扫到 0.10
    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    float prev_m1 = motors.m1;
    float max_thr_step = 0.0f;
    float m1_at_boundary = 0.0f;
    for (int i = 1; i <= 20; ++i) {
        ControlCommand_t cmd_sweep = {(float)i * 0.005f, 0.0f, 0.0f, 0.0f, true};
        controller_step(&fc, zero_gyro, flat_accel, cmd_sweep, 3.90f, 10U, &motors);
        if (i == 10) {
            m1_at_boundary = motors.m1;
        }
        float step_diff = fabsf(motors.m1 - prev_m1);
        if (step_diff > max_thr_step) {
            max_thr_step = step_diff;
        }
        prev_m1 = motors.m1;
    }
    CHECK(fabsf(m1_at_boundary - MOTOR_IDLE_DEFAULT) < 1e-5f && max_thr_step < 0.005f,
          "6. Throttle transition across 5% idle boundary is C0-continuous (max step < 0.005)");

    // 7. 满油门姿态控制余量测试（E-06）：100% 满油门下打 +15 度横滚，仍保有 >5% 左右差动余量
    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    ControlCommand_t cmd_full_roll = {1.00f, DEG2RAD(15.0f), 0.0f, 0.0f, true};
    controller_step(&fc, zero_gyro, flat_accel, cmd_full_roll, 3.90f, 10U, &motors);
    CHECK((motors.m3 - motors.m1) > 0.05f && (motors.m4 - motors.m2) > 0.05f,
          "7. Full throttle (100%) preserves >5% differential motor authority for attitude control");

    // 8. 实测 dt 钳位测试（C-03）：超大 dt (5.0s)、负值 dt (-1.0s) 与 NAN 均严格钳位至 [0.5, 1.5] * CTRL_DT
    FlightController_t fc_ref, fc_test;
    MotorOutput_t m_ref = {0}, m_test = {0};
    ControlCommand_t cmd_roll = {0.50f, DEG2RAD(10.0f), 0.0f, 0.0f, true};

    controller_init(&fc_ref);
    controller_set_gyro_bias(&fc_ref, zero_gyro);
    (void)controller_step(&fc_ref, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &m_ref);
    controller_step_dt(&fc_ref, zero_gyro, flat_accel, cmd_roll, 3.90f, 10U, 1.5f * CTRL_DT, &m_ref);

    controller_init(&fc_test);
    controller_set_gyro_bias(&fc_test, zero_gyro);
    (void)controller_step(&fc_test, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &m_test);
    controller_step_dt(&fc_test, zero_gyro, flat_accel, cmd_roll, 3.90f, 10U, 5.0f, &m_test);
    CHECK(fabsf(m_test.m1 - m_ref.m1) < 1e-6f && fabsf(m_test.m3 - m_ref.m3) < 1e-6f,
          "8a. Excessively large dt_sec (5.0s) is safely clamped to 1.5 * CTRL_DT");

    controller_init(&fc_ref);
    controller_set_gyro_bias(&fc_ref, zero_gyro);
    (void)controller_step(&fc_ref, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &m_ref);
    controller_step_dt(&fc_ref, zero_gyro, flat_accel, cmd_roll, 3.90f, 10U, 0.5f * CTRL_DT, &m_ref);

    controller_init(&fc_test);
    controller_set_gyro_bias(&fc_test, zero_gyro);
    (void)controller_step(&fc_test, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &m_test);
    controller_step_dt(&fc_test, zero_gyro, flat_accel, cmd_roll, 3.90f, 10U, -1.0f, &m_test);
    CHECK(fabsf(m_test.m1 - m_ref.m1) < 1e-6f && fabsf(m_test.m3 - m_ref.m3) < 1e-6f,
          "8b. Negative dt_sec (-1.0s) is safely clamped to 0.5 * CTRL_DT");

    controller_init(&fc_ref);
    controller_set_gyro_bias(&fc_ref, zero_gyro);
    (void)controller_step(&fc_ref, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &m_ref);
    controller_step(&fc_ref, zero_gyro, flat_accel, cmd_roll, 3.90f, 10U, &m_ref);

    controller_init(&fc_test);
    controller_set_gyro_bias(&fc_test, zero_gyro);
    (void)controller_step(&fc_test, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &m_test);
    controller_step_dt(&fc_test, zero_gyro, flat_accel, cmd_roll, 3.90f, 10U, NAN, &m_test);
    CHECK(fabsf(m_test.m1 - m_ref.m1) < 1e-6f && fabsf(m_test.m3 - m_ref.m3) < 1e-6f,
          "8c. Non-finite NAN dt_sec falls back to nominal CTRL_DT cleanly");

    // 9. NaN 断帧保护（F-02）：先收敛至 30 度倾角，再连续注入 40 拍全 NAN 坏帧，姿态不得被假水平拉偏
    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    ControlCommand_t cmd_locked_cal = {0.0f, 0.0f, 0.0f, 0.0f, false};
    Vector3_t tilt30_accel = {0.0f, 0.5f, 0.8660254f};
    for (int i = 0; i < 300; ++i) {
        (void)controller_step(&fc, zero_gyro, tilt30_accel, cmd_locked_cal, 3.90f, 10U, &motors);
    }
    (void)controller_step(&fc, zero_gyro, tilt30_accel, cmd_arm, 3.90f, 10U, &motors);
    float roll_before_nan_deg = RAD2DEG(fc.attitude.roll);
    Vector3_t nan_vec = {NAN, NAN, NAN};
    for (int i = 0; i < 40; ++i) {
        (void)controller_step(&fc, nan_vec, nan_vec, cmd_hover, 3.90f, 10U, &motors);
    }
    float roll_after_nan_deg = RAD2DEG(fc.attitude.roll);
    printf("F-02 NAN hold | Before=%.2f deg -> After 40 NAN ticks=%.2f deg\n",
           roll_before_nan_deg, roll_after_nan_deg);
    CHECK(roll_before_nan_deg > 25.0f && fabsf(roll_after_nan_deg - roll_before_nan_deg) < 1.0f,
          "9. 40 consecutive NAN frames hold tilt without fake-level Mahony pull (< 1.0 deg drift, F-02)");

    // 10. 内环 Roll/Pitch 积分权限验证（F-05）：controller_init 默认参数应具备配平偏心所需的积分权限
    controller_init(&fc);
    for (int i = 0; i < 1200; ++i) {
        (void)pid_update_measurement(&fc.rate_roll_pid, 0.0f, 0.2f, CTRL_DT);
        (void)pid_update_measurement(&fc.rate_pitch_pid, 0.0f, 0.2f, CTRL_DT);
    }
    CHECK(fabsf(fc.rate_roll_pid.integral * fc.rate_roll_pid.ki) >= 0.04f &&
          fabsf(fc.rate_pitch_pid.integral * fc.rate_pitch_pid.ki) >= 0.04f,
          "10. controller_init Roll/Pitch rate PIDs have >= 0.04 integral trim authority (F-05)");

    // 11. 5% 油门门槛差动权限渐入（F-06）：
    //   11a: throttle=0.051 + 25° roll 目标，四路差动极差应 < 0.06（docs/ERRATA.md 验收门限）
    //   11b: throttle=0.090 (mix_throttle=0.04) + 2° roll 目标（不触发保均值饱和），验证 auth=0.40 真实把极差压低至 < 0.04（旧代码 ≈ 0.069 必失败）
    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    ControlCommand_t cmd_thr_edge = {0.051f, DEG2RAD(25.0f), 0.0f, 0.0f, true};
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_thr_edge, 3.90f, 10U, &motors);
    float m_arr[4] = {motors.m1, motors.m2, motors.m3, motors.m4};
    float m_min = m_arr[0], m_max = m_arr[0];
    for (int i = 1; i < 4; ++i) {
        if (m_arr[i] < m_min) m_min = m_arr[i];
        if (m_arr[i] > m_max) m_max = m_arr[i];
    }
    printf("F-06 Edge Authority (thr=0.051, roll=25deg) | spread=%.4f (min=%.4f max=%.4f)\n",
           m_max - m_min, m_min, m_max);
    CHECK((m_max - m_min) < 0.06f,
          "11a. 5.1% throttle edge keeps differential motor spread < 0.06 (F-06)");

    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    ControlCommand_t cmd_thr_ramp = {0.090f, DEG2RAD(2.0f), 0.0f, 0.0f, true};
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_thr_ramp, 3.90f, 10U, &motors);
    float r_arr[4] = {motors.m1, motors.m2, motors.m3, motors.m4};
    float r_min = r_arr[0], r_max = r_arr[0];
    for (int i = 1; i < 4; ++i) {
        if (r_arr[i] < r_min) r_min = r_arr[i];
        if (r_arr[i] > r_max) r_max = r_arr[i];
    }
    printf("F-06 Ramp Authority (thr=0.090, roll=2deg)  | spread=%.4f (min=%.4f max=%.4f)\n",
           r_max - r_min, r_min, r_max);
    CHECK((r_max - r_min) < 0.04f,
          "11b. 9% throttle ramp attenuates unsat roll spread from ~0.069 to < 0.04 (F-06)");

    // 12. 故障必须锁存：覆盖单独陀螺仪失效、单独加计失效及全帧失效。
    for (int fault = 0; fault < 3; ++fault) {
        controller_init(&fc);
        controller_set_gyro_bias(&fc, zero_gyro);
        (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.9f, 0U, &motors);
        Vector3_t bad_gyro = fault == 1 ? zero_gyro : nan_vec;
        Vector3_t bad_accel = fault == 0 ? flat_accel : nan_vec;
        for (unsigned i = 1; i < CTRL_IMU_FAILURE_SAMPLES; ++i) {
            st = controller_step(&fc, bad_gyro, bad_accel, cmd_hover, 3.9f, 0U, &motors);
        }
        CHECK(st == FLIGHT_STATE_ARMED && isfinite(fc.attitude.roll),
              "12a. Transient invalid frames preserve finite AHRS before failure threshold");
        st = controller_step(&fc, bad_gyro, bad_accel, cmd_hover, 3.9f, 0U, &motors);
        CHECK(st == FLIGHT_STATE_FAILSAFE && motors.m1 == 0 && motors.m2 == 0 &&
              motors.m3 == 0 && motors.m4 == 0,
              "12b. 40th invalid IMU frame latches FAILSAFE and stops all motors");
        st = controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.9f, 0U, &motors);
        CHECK(st == FLIGHT_STATE_FAILSAFE && motors.m1 == 0 && motors.m2 == 0 &&
              motors.m3 == 0 && motors.m4 == 0,
              "12c. IMU recovery with armed=true and low throttle cannot restart motors");
        st = controller_step(&fc, zero_gyro, flat_accel, cmd_locked_cal, 3.9f, 0U, &motors);
        CHECK(st == FLIGHT_STATE_LOCKED, "12d. Explicit pilot disarm clears fault latch");
        st = controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.9f, 0U, &motors);
        CHECK(st == FLIGHT_STATE_ARMED && fabsf(motors.m1 - MOTOR_IDLE_DEFAULT) < 1e-6f,
              "12e. Healthy IMU permits rearming after explicit disarm");
    }

    // 13. 校准必须由连续有效帧构成，坏帧不可拼接两段采样。
    controller_init(&fc);
    for (unsigned i = 0; i < CTRL_GYRO_CAL_SAMPLES - 1U; ++i) {
        (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.9f, 0U, &motors);
    }
    (void)controller_step(&fc, nan_vec, flat_accel, cmd_arm, 3.9f, 0U, &motors);
    CHECK(fc.cal_count == 0U && !fc.gyro_calibrated,
          "13a. Invalid gyro interrupts calibration and resets sample count");
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.9f, 0U, &motors);
    (void)controller_step(&fc, zero_gyro, nan_vec, cmd_arm, 3.9f, 0U, &motors);
    CHECK(fc.cal_count == 0U && !fc.gyro_calibrated,
          "13b. Invalid accelerometer also interrupts stationary calibration");

    // 14. ADC 零读数不能让历史正常电压永久掩盖采样断线或真实电压异常。
    controller_init(&fc);
    controller_set_gyro_bias(&fc, zero_gyro);
    (void)controller_step(&fc, zero_gyro, flat_accel, cmd_arm, 3.9f, 0U, &motors);
    st = controller_step(&fc, zero_gyro, flat_accel, cmd_hover, 0.0f, 0U, &motors);
    CHECK(st == FLIGHT_STATE_ARMED && fc.vbat_filtered > SAFETY_VBAT_WARN_V,
          "14a. Battery filter rejects a single zero ADC glitch");
    for (int i = 0; i < 32; ++i) {
        st = controller_step(&fc, zero_gyro, flat_accel, cmd_hover, 0.0f, 0U, &motors);
    }
    CHECK(st == FLIGHT_STATE_LOWBAT_LAND && fc.vbat_filtered < SAFETY_VBAT_LAND_V,
          "14b. Sustained zero ADC reading triggers low battery landing");
    st = controller_step(&fc, zero_gyro, flat_accel, cmd_hover, 3.9f, 0U, &motors);
    CHECK(st == FLIGHT_STATE_LOWBAT_LAND,
          "14c. ADC recovery does not release low battery latch");

    printf("=================================================================\n");
    if (g_failures == 0) {
        printf("ALL CONTROLLER TESTS PASSED!\n");
        return 0;
    }
    printf("CONTROLLER TESTS FAILED: %d\n", g_failures);
    return 1;
}
