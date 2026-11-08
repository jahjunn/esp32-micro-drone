#pragma once

#include <Arduino.h>

/*
 * 1S 锂电池电压采集驱动 (ADC1_CH4 / GPIO5，100k / 100k 分压)
 * 返回经出厂 eFuse 校准后的电池电压，单位 [V]；低通抗毛刺滤波由上层 controller_step 完成。
 */

void  vbat_init(void);
float vbat_read_volts(void);
