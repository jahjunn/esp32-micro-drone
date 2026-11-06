#include <stdio.h>
#include <math.h>
#include "../src/flight/mixer.h"

static int g_fails = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] line %d: %s\n", __LINE__, (msg)); \
        g_fails++; \
    } else { \
        printf("[PASS] %s\n", (msg)); \
    } \
} while (0)

static int nearf(float a, float b, float eps) {
    return fabsf(a - b) <= eps;
}

static void show(const char *name, float throttle, float roll, float pitch, float yaw, MotorOutput_t *out) {
    mixer_quad_x(throttle, roll, pitch, yaw, out);
    printf("%-14s | M1=%.2f M2=%.2f M3=%.2f M4=%.2f\n",
           name, out->m1, out->m2, out->m3, out->m4);
}

int main(void) {
    MotorOutput_t o;
    printf("================ Mixer & Convention Test ================\n");

    // 1. 悬停：四路电机输出完全相等 = 0.50
    show("hover", 0.50f, 0.00f, 0.00f, 0.00f, &o);
    CHECK(nearf(o.m1, 0.50f, 1e-5f) && nearf(o.m2, 0.50f, 1e-5f) &&
          nearf(o.m3, 0.50f, 1e-5f) && nearf(o.m4, 0.50f, 1e-5f),
          "hover: all 4 motors equal 0.50");

    // 2. 右倾力矩 (+roll = +0.10)：左侧 M3/M4 必须加速(0.60)，右侧 M1/M2 必须减速(0.40)
    show("roll right(+)", 0.50f, 0.10f, 0.00f, 0.00f, &o);
    CHECK(o.m3 > o.m1 && o.m4 > o.m2 && nearf(o.m3, 0.60f, 1e-5f) && nearf(o.m1, 0.40f, 1e-5f),
          "roll +0.10 (right): left motors (M3,M4) faster than right (M1,M2)");

    // 3. 抬头力矩 (+pitch = +0.10)：前侧 M1/M4 必须加速(0.60)，后侧 M2/M3 必须减速(0.40)
    show("pitch up(+)", 0.50f, 0.00f, 0.10f, 0.00f, &o);
    CHECK(o.m1 > o.m2 && o.m4 > o.m3 && nearf(o.m1, 0.60f, 1e-5f) && nearf(o.m2, 0.40f, 1e-5f),
          "pitch +0.10 (nose-up): front motors (M1,M4) faster than rear (M2,M3)");

    // 4. 顺时针右转 (+yaw = +0.10)：CCW 桨 M1/M3 必须加速(0.60)，CW 桨 M2/M4 必须减速(0.40)
    show("yaw cw(+)", 0.50f, 0.00f, 0.00f, 0.10f, &o);
    CHECK(o.m1 > o.m2 && o.m3 > o.m4 && nearf(o.m1, 0.60f, 1e-5f) && nearf(o.m2, 0.40f, 1e-5f),
          "yaw +0.10 (CW): CCW motors (M1,M3) faster than CW (M2,M4)");

    // 5. 高油门抗饱和测试：上限不超过 1.00，且四路均值严格保持为 0.95
    show("overflow", 0.95f, 0.20f, 0.00f, 0.00f, &o);
    float mean_hi = (o.m1 + o.m2 + o.m3 + o.m4) * 0.25f;
    CHECK(o.m3 <= 1.0f && o.m4 <= 1.0f && nearf(mean_hi, 0.95f, 1e-5f),
          "overflow desaturate: max <= 1.0 and mean preserved at 0.95");

    // 6. 低油门抗饱和 + 电机怠速保底测试：经 mixer_apply_idle 后所有电机 >= 0.12
    show("underflow", 0.05f, 0.20f, 0.00f, 0.00f, &o);
    mixer_apply_idle(&o, MOTOR_IDLE_DEFAULT);
    printf("after idle     | M1=%.2f M2=%.2f M3=%.2f M4=%.2f\n", o.m1, o.m2, o.m3, o.m4);
    CHECK(o.m1 >= MOTOR_IDLE_DEFAULT - 1e-5f && o.m2 >= MOTOR_IDLE_DEFAULT - 1e-5f &&
          o.m3 >= MOTOR_IDLE_DEFAULT - 1e-5f && o.m4 >= MOTOR_IDLE_DEFAULT - 1e-5f,
          "motor idle floor: all motors >= MOTOR_IDLE_DEFAULT (0.12)");

    printf("=========================================================\n");
    if (g_fails == 0) {
        printf("ALL MIXER TESTS PASSED!\n");
        return 0;
    } else {
        printf("MIXER TESTS FAILED: %d error(s)\n", g_fails);
        return 1;
    }
}