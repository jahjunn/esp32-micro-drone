#include "status_led.h"
#include "../config.h"

static inline void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(PIN_STATUS_LED, r, g, b);
}

void status_led_init(void) {
    set_rgb(0U, 0U, 20U);
}

void status_led_update(FlightState_t state, bool calibrating, bool ble_connected, bool imu_ok) {
    uint32_t now = millis();

    // 1. 硬件 IMU 初始化或通信严重异常
    if (!imu_ok) {
        if (((now / 120U) % 2U) == 0U) {
            set_rgb(48U, 0U, 0U);
        } else {
            set_rgb(36U, 24U, 0U);
        }
        return;
    }

    // 2. E-03：正在等待陀螺仪 200 拍静止校准 -> 青蓝 5Hz 快闪提示“请平放勿触碰”
    if (calibrating) {
        if (((now / 100U) % 2U) == 0U) {
            set_rgb(0U, 28U, 40U);
        } else {
            set_rgb(0U, 0U, 0U);
        }
        return;
    }

    // 3. 飞控五态安全灯语
    switch (state) {
        case FLIGHT_STATE_FAILSAFE:
            if (((now / 50U) % 2U) == 0U) {
                set_rgb(48U, 0U, 0U);
            } else {
                set_rgb(0U, 0U, 0U);
            }
            break;

        case FLIGHT_STATE_LOWBAT_LAND:
            if (((now / 100U) % 2U) == 0U) {
                set_rgb(44U, 8U, 0U);
            } else {
                set_rgb(0U, 0U, 0U);
            }
            break;

        case FLIGHT_STATE_LOWBAT_WARN:
            if (((now / 200U) % 2U) == 0U) {
                set_rgb(36U, 20U, 0U);
            } else {
                set_rgb(0U, 0U, 0U);
            }
            break;

        case FLIGHT_STATE_ARMED:
            set_rgb(0U, 32U, 0U);
            break;

        case FLIGHT_STATE_LOCKED:
        default:
            if (ble_connected) {
                set_rgb(0U, 18U, 14U); // BLE 已连接待命
            } else {
                // 等待手机连接：1Hz 蓝色心跳闪
                if ((now % 1000U) < 250U) {
                    set_rgb(0U, 12U, 36U);
                } else {
                    set_rgb(0U, 0U, 0U);
                }
            }
            break;
    }
}
