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

    printf("================ Cascaded Flight Controller Test ================\n");

    // 1. 测试陀螺仪未完成校准前禁止解锁 + 手抖自动重测 + 200 拍静止校准通过
    ControlCommand_t cmd_arm = {0.0f, 0.0f, 0.0f, 0.0f, true};
    Vector3_t biased_gyro = {DEG2RAD(2.5f), DEG2RAD(-1.5f), DEG2RAD(0.8f)};

    // 先采 50 拍正常数据，第 51 拍模拟手抖了一下 (0.5 rad/s > 0.25 rad/s)，验证校准计数被重置为 0
    for (int i = 0; i < 50; ++i) {
        controller_step(&fc, biased_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    }
    Vector3_t shake_gyro = {0.50f, 0.0f, 0.0f};
    FlightState_t st = controller_step(&fc, shake_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    CHECK(!fc.gyro_calibrated && fc.cal_count == 0U && st == FLIGHT_STATE_LOCKED,
          "1a. Hand shake during gyro calibration resets sample counter & blocks arming");

    // 随后模拟桌面存在轻微共振（±0.09 rad/s，峰峰值 0.18 <= 0.25 容差），验证可顺利完成校准并准确算出零偏
    CHECK(controller_is_calibrating(&fc),
          "1b. controller_is_calibrating() reports true before calibration completes");
    for (int i = 0; i < (int)CTRL_GYRO_CAL_SAMPLES; ++i) {
        float micro_vib = (i % 2 == 0) ? 0.09f : -0.09f;
        Vector3_t desk_gyro = {biased_gyro.x + micro_vib, biased_gyro.y, biased_gyro.z};
        st = controller_step(&fc, desk_gyro, flat_accel, cmd_arm, 3.90f, 10U, &motors);
    }
    CHECK(!controller_is_calibrating(&fc) &&
          fabsf(fc.gyro_bias.x - biased_gyro.x) < 1e-5f &&
          fabsf(fc.gyro_bias.y - biased_gyro.y) < 1e-5f,
          "1c. 200 ticks with 0.18 rad/s desk micro-vibration complete calibration accurately");

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

    printf("=================================================================\n");
    if (g_failures == 0) {
        printf("ALL CONTROLLER TESTS PASSED!\n");
        return 0;
    }
    printf("CONTROLLER TESTS FAILED: %d\n", g_failures);
    return 1;
}
