#include <math.h>
#include <stdio.h>
#include "../src/comm/protocol.h"
#include "../src/flight/controller.h"

#define RAD(deg) ((deg) * ((float)M_PI / 180.0f))
#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        fprintf(stderr, "[FAIL] %s (line %d)\n", msg, __LINE__); \
        return 1; \
    } \
} while (0)

static bool stopped(MotorOutput_t m) {
    return m.m1 == 0.0f && m.m2 == 0.0f && m.m3 == 0.0f && m.m4 == 0.0f;
}

static bool within_idle_range(MotorOutput_t m) {
    const float floor = MOTOR_IDLE_DEFAULT - 1e-6f;
    return m.m1 >= floor && m.m1 <= 1.0f && m.m2 >= floor && m.m2 <= 1.0f &&
           m.m3 >= floor && m.m3 <= 1.0f && m.m4 >= floor && m.m4 <= 1.0f;
}

static int run_startup_cases(void) {
    FlightController_t fc;
    MotorOutput_t motor;
    Vector3_t level = {0.0f, 0.0f, 1.0f}, inverted = {0.0f, 0.0f, -1.0f};
    Vector3_t zero = {0}, invalid = {NAN, NAN, NAN};
    ControlCommand_t arm = {0.0f, 0.0f, 0.0f, 0.0f, true};
    controller_init(&fc);
    for (unsigned i = 0; i <= CTRL_GYRO_CAL_SAMPLES; ++i) {
        CHECK(controller_step(&fc, zero, inverted, arm, 3.9f, 0, &motor) == FLIGHT_STATE_LOCKED &&
              stopped(motor), "inverted startup cannot arm even after gyro calibration");
    }
    CHECK(cosf(fc.attitude.roll) * cosf(fc.attitude.pitch) < -0.99f,
          "startup estimator recognizes inverted gravity");

    controller_init(&fc);
    for (unsigned i = 0; i <= CTRL_GYRO_CAL_SAMPLES; ++i) {
        CHECK(controller_step(&fc, zero, zero, arm, 3.9f, 0, &motor) == FLIGHT_STATE_LOCKED &&
              stopped(motor), "zero-g samples do not satisfy stationary startup");
    }
    CHECK(controller_is_calibrating(&fc), "zero-g frames cannot complete calibration");
    for (unsigned i = 0; i < CTRL_GYRO_CAL_SAMPLES; ++i) {
        controller_step(&fc, zero, level, arm, 3.9f, 0, &motor);
    }
    CHECK(controller_step(&fc, invalid, level, arm, 3.9f, 0, &motor) == FLIGHT_STATE_LOCKED &&
          stopped(motor), "a dropped IMU sample cannot authorize arming");
    CHECK(controller_step(&fc, zero, level, arm, 3.9f, 0, &motor) == FLIGHT_STATE_ARMED,
          "valid IMU frame permits low-throttle arming");

    controller_init(&fc);
    const float roll = RAD(35.0f), pitch = RAD(-25.0f);
    Vector3_t tilted = {-sinf(pitch), sinf(roll) * cosf(pitch), cosf(roll) * cosf(pitch)};
    for (unsigned i = 0; i < CTRL_GYRO_CAL_SAMPLES; ++i) {
        controller_step(&fc, zero, tilted, arm, 3.9f, 0, &motor);
    }
    CHECK(fabsf(fc.attitude.roll - roll) < RAD(0.1f) && fabsf(fc.attitude.pitch - pitch) < RAD(0.1f),
          "startup alignment estimates both FRD tilt signs on a stationary surface");
    return 0;
}

static int run_attitude_case(float direction, float motor_lag_s, float torque_gain) {
    FlightController_t fc;
    controller_init(&fc);
    MotorOutput_t motor = {0};
    Vector3_t zero = {0};
    Vector3_t level = {0.0f, 0.0f, 1.0f};
    ControlCommand_t arm = {0};
    for (unsigned i = 0; i < CTRL_GYRO_CAL_SAMPLES; ++i) {
        CHECK(controller_step(&fc, zero, level, arm, 3.9f, 0, &motor) == FLIGHT_STATE_LOCKED &&
              stopped(motor), "startup calibration holds all outputs at zero");
    }
    arm.armed = true;
    CHECK(controller_step(&fc, zero, level, arm, 3.9f, 0, &motor) == FLIGHT_STATE_ARMED &&
          within_idle_range(motor), "fresh low-throttle command arms after calibration");

    float roll = 0.0f, pitch = 0.0f;
    float roll_rate = 0.0f, pitch_rate = 0.0f;
    float roll_moment = 0.0f, pitch_moment = 0.0f;
    float peak_axis_angle = 0.0f;
    ControlCommand_t cmd = arm;
    const unsigned frame_divider = 8;  // 50Hz sender and 400Hz nominal control.
    for (unsigned n = 0; n < 1600; ++n) {
        if (n % frame_divider == 0) {
            // Produce the same wire frame a 50Hz sender would produce.
            ControlCommand_t pilot = {0.50f, n < 400 ? direction * RAD(8.0f) : 0.0f,
                                      n < 400 ? -direction * RAD(6.0f) : 0.0f, 0.0f, true};
            BleControlPacket_t packet;
            protocol_pack_control(&pilot, (uint8_t)(n / frame_divider), 0, &packet);
            CHECK(protocol_parse_control((const uint8_t *)&packet, sizeof(packet), &cmd, NULL) ==
                  PROTOCOL_OK, "valid control frame reaches the firmware controller");
        }
        if (n == 640) {
            roll_rate += direction * RAD(100.0f);
            pitch_rate += direction * RAD(80.0f);
        }
        float dt = CTRL_DT * (n % 2 ? 1.1f : 0.9f);
        // Simplified roll/pitch test plant with damped rates and first-order motor lag.
        // Gravity and gyro are synthesized from the plant in sensor FLU coordinates.
        Vector3_t accel_flu = {sinf(pitch), sinf(roll) * cosf(pitch), cosf(roll) * cosf(pitch)};
        Vector3_t gyro_flu = {roll_rate, -pitch_rate * cosf(roll), pitch_rate * sinf(roll)};
        Vector3_t gyro, accel;
        controller_map_flu_to_frd(gyro_flu, accel_flu, &gyro, &accel);
        if (n == 640 || n == 641) gyro = accel = (Vector3_t){NAN, NAN, NAN};
        CHECK(controller_step_dt(&fc, gyro, accel, cmd, 3.9f,
                                 (n % frame_divider) * 5 / 2, dt, &motor) == FLIGHT_STATE_ARMED &&
              within_idle_range(motor), "transient dropouts and jitter retain bounded outputs");
        float roll_cmd = torque_gain * (motor.m3 + motor.m4 - motor.m1 - motor.m2);
        float pitch_cmd = torque_gain * (motor.m1 + motor.m4 - motor.m2 - motor.m3);
        float response = dt / (motor_lag_s + dt);
        roll_moment += response * (roll_cmd - roll_moment);
        pitch_moment += response * (pitch_cmd - pitch_moment);
        roll_rate += (roll_moment - 2.0f * roll_rate) * dt;
        pitch_rate += (pitch_moment - 2.0f * pitch_rate) * dt;
        roll += roll_rate * dt;
        pitch += pitch_rate * dt;
        peak_axis_angle = fmaxf(peak_axis_angle, fmaxf(fabsf(roll), fabsf(pitch)));
        CHECK(fabsf(roll) < RAD(25.0f) && fabsf(pitch) < RAD(25.0f),
              "step and gust stay inside the synthetic test envelope");
        if (n == 399) {
            CHECK(direction * roll > RAD(5.0f) && -direction * pitch > RAD(4.0f),
                  "tracking follows both FRD command signs");
        }
    }
    CHECK(fabsf(roll) < RAD(1.0f) && fabsf(pitch) < RAD(1.0f) &&
          fabsf(fc.attitude.roll - roll) < RAD(1.0f) && fabsf(fc.attitude.pitch - pitch) < RAD(1.0f),
          "both axes return to level after the gust");

    // Supply droop latches landing, then the BLE callback's stale disarm must
    // produce FAILSAFE; a fresh arm alone cannot reset it after reconnection.
    for (unsigned i = 0; i < 120; ++i) {
        controller_step(&fc, zero, level, cmd, 3.1f, 0, &motor);
    }
    CHECK(fc.safety.state == FLIGHT_STATE_LOWBAT_LAND, "sustained filtered low battery latches");
    CHECK(controller_step(&fc, zero, level, cmd, 3.9f, 0, &motor) == FLIGHT_STATE_LOWBAT_LAND,
          "battery rebound preserves the landing latch");
    ControlCommand_t disarm = {0};
    CHECK(controller_step(&fc, zero, level, disarm, 3.9f, SAFETY_TIMEOUT_MS + 1, &motor) ==
          FLIGHT_STATE_FAILSAFE && stopped(motor), "BLE disconnect is a fault, not a pilot reset");
    CHECK(controller_step(&fc, zero, level, arm, 3.9f, 0, &motor) == FLIGHT_STATE_FAILSAFE &&
          stopped(motor), "reconnection carrying armed=true cannot restart motors");
    CHECK(controller_step(&fc, zero, level, disarm, 3.9f, 0, &motor) == FLIGHT_STATE_LOCKED &&
          stopped(motor), "fresh disarm acknowledges the fault");
    const float deg_per_rad = 180.0f / (float)M_PI;
    printf("[PASS] direction=%+.0f lag=%.0fms gain=%.0f peak=%.2fdeg final=(%+.2f, %+.2f)deg\n",
           direction, 1000.0f * motor_lag_s, torque_gain, peak_axis_angle * deg_per_rad,
           roll * deg_per_rad, pitch * deg_per_rad);
    return 0;
}

int main(void) {
    unsigned failures = (unsigned)run_startup_cases();
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned lag = 0; lag <= 2; ++lag) {
            for (unsigned gain = 0; gain <= 2; ++gain) {
                if (run_attitude_case((float)direction, lag * 0.02f, 20.0f + gain * 5.0f)) {
                    fprintf(stderr, "Case: direction=%+d lag=%ums gain=%u\n",
                            direction, lag * 20U, 20U + gain * 5U);
                    ++failures;
                }
            }
        }
    }
    CHECK(failures == 0, "startup cases and all 18 deterministic flight scenarios pass");
    puts("[PASS] Startup interlocks and 18 synthetic roll/pitch, power and link-loss scenarios");
    return 0;
}
