#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "../flight/types.h"

/*
 * 四路 8520 有刷电机高频 LEDC PWM 输出驱动
 *
 * 【硬件参数】：
 *   - 20 kHz / 10-bit (0 ~ 1023)，低边 AO3400 NMOS 驱动 + SS34 续流二极管；
 *   - 上电初始化先将 LEDC 通道清零再绑定 GPIO，杜绝开机毛刺引发电机误转。
 */

void motor_out_init(void);
void motor_out_write(MotorOutput_t out);
void motor_out_stop_all(void);
