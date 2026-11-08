#pragma once

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include "../flight/safety.h"

/*
 * 板载 WS2812 RGB 状态指示灯驱动 (GPIO48)
 *
 * 【灯语约定（建议 20Hz 在 Core0 刷新）】：
 *   - 红/黄快闪       ：MPU6050 硬件故障，停飞排查 I2C；
 *   - 青蓝色 5Hz 快闪 ：开机陀螺仪静止校准中（E-03：手勿碰机，平放约 0.5 秒）；
 *   - 蓝色慢闪 (1Hz)  ：LOCKED 待连接 BLE；
 *   - 青绿常亮        ：LOCKED 且 BLE 已连接，可长按解锁；
 *   - 纯绿常亮        ：ARMED 正常解锁飞行；
 *   - 琥珀黄闪烁      ：LOWBAT_WARN (<3.5V) 请返航；
 *   - 橙红急闪        ：LOWBAT_LAND (<3.3V) 强制限高缓降；
 *   - 红灯爆闪 (10Hz) ：FAILSAFE 故障锁死（需手柄拨回上锁复位）。
 */

void status_led_init(void);
void status_led_update(FlightState_t state, bool calibrating, bool ble_connected, bool imu_ok);
