#include "motor_out.h"
#include "../config.h"
#include <math.h>

static inline uint32_t duty_from_norm(float val) {
    if (!isfinite(val) || val <= 0.0f) {
        return 0U;
    }
    if (val >= 1.0f) {
        return MOTOR_PWM_MAX_DUTY;
    }
    uint32_t duty = (uint32_t)lroundf(val * (float)MOTOR_PWM_MAX_DUTY);
    return (duty > MOTOR_PWM_MAX_DUTY) ? MOTOR_PWM_MAX_DUTY : duty;
}

void motor_out_init(void) {
    // 先将输出电平设为 LOW 再配置 20kHz LEDC 定时器，确保启动过程零瞬态高电平
    const uint8_t pins[4]     = { PIN_MOTOR_M1, PIN_MOTOR_M2, PIN_MOTOR_M3, PIN_MOTOR_M4 };
    const uint8_t channels[4] = { LEDC_CH_M1,   LEDC_CH_M2,   LEDC_CH_M3,   LEDC_CH_M4   };

    for (int i = 0; i < 4; ++i) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
        ledcSetup(channels[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
        ledcWrite(channels[i], 0U);
        ledcAttachPin(pins[i], channels[i]);
    }
}

void motor_out_write(MotorOutput_t out) {
    ledcWrite(LEDC_CH_M1, duty_from_norm(out.m1));
    ledcWrite(LEDC_CH_M2, duty_from_norm(out.m2));
    ledcWrite(LEDC_CH_M3, duty_from_norm(out.m3));
    ledcWrite(LEDC_CH_M4, duty_from_norm(out.m4));
}

void motor_out_stop_all(void) {
    ledcWrite(LEDC_CH_M1, 0U);
    ledcWrite(LEDC_CH_M2, 0U);
    ledcWrite(LEDC_CH_M3, 0U);
    ledcWrite(LEDC_CH_M4, 0U);
}
