#pragma once

/*
 * config.h —— ESP32-S3-N16R8 轻量四轴飞控硬件引脚与任务周期配置（对齐 docs/ARCHITECTURE.md §9.2）
 *
 * 【硬件红线注意】：
 *   - GPIO26~32 被 16MB Quad Flash 占用、GPIO35~37 被 8MB OPI PSRAM 占用，严禁复用；
 *   - GPIO19/20 为原生 USB D-/D+，GPIO43/44 为 UART0，严禁用于电机 PWM。
 */

// ==================== 1. 电机 PWM 引脚与 LEDC 参数 (Low-Side AO3400) ====================
#define PIN_MOTOR_M1          1       // 右前 (CCW) — LEDC CH0
#define PIN_MOTOR_M2          2       // 右后 (CW)  — LEDC CH1
#define PIN_MOTOR_M3          3       // 左后 (CCW) — LEDC CH2
#define PIN_MOTOR_M4          4       // 左前 (CW)  — LEDC CH3

#define LEDC_CH_M1            0
#define LEDC_CH_M2            1
#define LEDC_CH_M3            2
#define LEDC_CH_M4            3

#define MOTOR_PWM_FREQ_HZ     20000U  // 20 kHz 高频有刷 PWM（超越人耳听觉，消啸叫且减小纹波）
#define MOTOR_PWM_RES_BITS    10U     // 10-bit 分辨率：0 ~ 1023
#define MOTOR_PWM_MAX_DUTY    ((1U << MOTOR_PWM_RES_BITS) - 1U)

// ==================== 2. I2C 传感器总线 (MPU6050 + QMC5883P) ====================
#define PIN_I2C_SDA           8
#define PIN_I2C_SCL           9
#define I2C_CLOCK_HZ          400000U // 400 kHz Fast Mode
#define I2C_TIMEOUT_MS        5U      // 超时保护：防 I2C 瞬态挂死拖拽 400Hz 控制环

#define MPU6050_I2C_ADDR      0x68    // AD0 接地默认地址

// ==================== 3. 电源电压采集 (1S LiPo 100k / 100k 分压) ====================
#define PIN_VBAT_ADC          5       // GPIO5 = ADC1_CH4（分压 + 100nF 退耦电容）
#define VBAT_DIVIDER_RATIO    2.0f    // 分压比 (Rtop + Rbot) / Rbot；可按实测万用表微调（如 2.02f）

// ==================== 4. 板载 WS2812 状态灯 ====================
#define PIN_STATUS_LED        48      // ESP32-S3-DevKitC-1 板载 RGB

// ==================== 5. FreeRTOS 双核与调度频率 ====================
#define UAV_CORE_COMM         0       // Core0：BLE GATT + 遥测推送 + 状态灯
#define UAV_CORE_CTRL         1       // Core1：400Hz 实时惯导采样 + 串级 PID + 混控 PWM

#define CTRL_LOOP_PERIOD_US   2500ULL // 400 Hz = 2500 us 硬件高精度周期定时器
#define CTRL_DT_MIN_SCALE     0.5f    // 实测 dt 下限钳位 (0.5 * 2.5ms = 1.25ms)
#define CTRL_DT_MAX_SCALE     1.5f    // 实测 dt 上限钳位 (1.5 * 2.5ms = 3.75ms)

#define TELEM_TASK_PERIOD_MS  50U     // Core0 遥测上行频率：20 Hz
#define LED_TASK_PERIOD_MS    50U     // Core0 状态灯刷新频率：20 Hz（支持 1~10Hz 各类告警闪烁）

// ==================== 6. 串口调试配置 ====================
#define SERIAL_BAUD_RATE      115200UL
