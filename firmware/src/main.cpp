/*
 * main.cpp —— ESP32-S3-N16R8 轻量四轴飞控双核主程序入口
 *
 * 【双核职责分工（FreeRTOS）】：
 *   - Core1（UAV_CORE_CTRL = 1，优先级 5）：
 *       由 400Hz (2500 us) esp_timer 高精度周期唤醒 vControlTask：
 *       实测循环 dt (C-03) -> 读 BLE 零积压指令 -> 读 MPU6050 (FLU) -> FLU->FRD 映射
 *       -> controller_step_dt (80Hz/40Hz LPF + Mahony + safety + 100Hz/400Hz 串级 PID + Quad-X + 怠速)
 *       -> 20kHz 10-bit LEDC 电机输出。
 *   - Core0（UAV_CORE_COMM = 0，优先级 2）：
 *       运行 BLE GATT 服务端回调，并以 20Hz 运行 vCommTask 推送 19B 遥测、刷新 GPIO48 WS2812 状态灯 (E-03) 与串口诊错。
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "flight/controller.h"
#include "comm/protocol.h"
#include "comm/ble_server.h"
#include "drivers/mpu6050.h"
#include "drivers/motor_out.h"
#include "drivers/vbat.h"
#include "drivers/status_led.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795130823208768f
#endif

// ==================== Core1 -> Core0 跨核零阻塞遥测快照 ====================
typedef struct {
    FlightState_t state;
    float         vbat_v;
    EulerAngle_t  attitude;
    MotorOutput_t motors;
    uint16_t      loop_us;
    bool          calibrating;
    bool          imu_ok;
    Vector3_t     gyro_bias;
} SharedTelem_t;

static portMUX_TYPE      s_telem_mux       = portMUX_INITIALIZER_UNLOCKED;
static SharedTelem_t     s_telem_snap      = {
    FLIGHT_STATE_LOCKED, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, 0U, true, false, {0.0f, 0.0f, 0.0f}
};

static FlightController_t s_fc;
static bool               s_imu_boot_ok    = false;
static TaskHandle_t       s_ctrl_task      = nullptr;
static esp_timer_handle_t s_ctrl_timer     = nullptr;

// ==================== 400Hz 硬件周期定时器回调（消除 FreeRTOS 1ms tick 截断误差） ====================
static void ctrl_timer_cb(void *arg) {
    (void)arg;
    if (s_ctrl_task != nullptr) {
        xTaskNotifyGive(s_ctrl_task);
    }
}

// ==================== Core1: 400Hz 实时惯导采样与飞行控制主循环 ====================
static void vControlTask(void *pvParameters) {
    (void)pvParameters;

    int64_t  prev_step_us   = esp_timer_get_time();
    uint32_t consec_imu_err = 0U;

    for (;;) {
        // 阻塞等待精确 2500 us 周期信号，空闲 CPU 期间释放给 Core1 IDLE（保障 Watchdog 喂狗）
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        int64_t now_us = esp_timer_get_time();
        float   dt_sec = (float)(now_us - prev_step_us) * 1e-6f;
        prev_step_us   = now_us;

        // 1. 无阻塞读取 Core0 BLE 最新遥控指令与指令包龄 (<1us)
        ControlCommand_t cmd        = { 0.0f, 0.0f, 0.0f, 0.0f, false };
        uint32_t         cmd_age_ms = 99999U;
        ble_comm_get_command(&cmd, &cmd_age_ms);

        // 2. 读取 1S 分压电池电压 [V]
        float vbat_v = vbat_read_volts();

        // 3. 若启动阶段 MPU6050 硬件失联，严禁输出任何电机 PWM 并锁死报错
        if (!s_imu_boot_ok) {
            motor_out_stop_all();
            portENTER_CRITICAL(&s_telem_mux);
            s_telem_snap.state       = FLIGHT_STATE_FAILSAFE;
            s_telem_snap.vbat_v      = vbat_v;
            s_telem_snap.motors      = { 0.0f, 0.0f, 0.0f, 0.0f };
            s_telem_snap.loop_us     = (uint16_t)(esp_timer_get_time() - now_us);
            s_telem_snap.calibrating = false;
            s_telem_snap.imu_ok      = false;
            portEXIT_CRITICAL(&s_telem_mux);
            continue;
        }

        // 4. 突发读取 MPU6050 原生 FLU 数据并映射为飞控标准 FRD 右手系
        Vector3_t gyro_flu  = { 0.0f, 0.0f, 0.0f };
        Vector3_t accel_flu = { 0.0f, 0.0f, 0.0f };
        Vector3_t gyro_frd  = { NAN,  NAN,  NAN  };
        Vector3_t accel_frd = { NAN,  NAN,  NAN  };

        if (mpu6050_read(&gyro_flu, &accel_flu, Wire, MPU6050_I2C_ADDR)) {
            consec_imu_err = 0U;
            controller_map_flu_to_frd(gyro_flu, accel_flu, &gyro_frd, &accel_frd);
        } else {
            // 单拍短暂 I2C 干扰传入 NAN，交由 controller.h NaN 防火墙保护滤波器不中毒；
            // 若连续 >= 40 拍 (100ms) 硬件掉线，立即强制撤销解锁权限防盲飞失控！
            if (consec_imu_err < 1000U) {
                consec_imu_err++;
            }
            if (consec_imu_err >= 40U) {
                cmd.armed    = false;
                cmd.throttle = 0.0f;
            }
        }

        // 5. 执行 400Hz 串级飞控步进（含 C-03 实测 dt 安全钳位）
        MotorOutput_t motors = { 0.0f, 0.0f, 0.0f, 0.0f };
        FlightState_t state  = controller_step_dt(
            &s_fc, gyro_frd, accel_frd, cmd, vbat_v, cmd_age_ms, dt_sec, &motors
        );

        // 6. 底层硬件总闸门写入 20kHz PWM
        bool imu_runtime_ok = (consec_imu_err < 40U);
        if (!imu_runtime_ok || state == FLIGHT_STATE_LOCKED || state == FLIGHT_STATE_FAILSAFE) {
            motor_out_stop_all();
            motors = { 0.0f, 0.0f, 0.0f, 0.0f };
        } else {
            motor_out_write(motors);
        }

        uint16_t exec_us = (uint16_t)(esp_timer_get_time() - now_us);

        // 7. 原子发布最新遥测快照给 Core0
        portENTER_CRITICAL(&s_telem_mux);
        s_telem_snap.state       = state;
        s_telem_snap.vbat_v      = s_fc.vbat_filtered > 0.1f ? s_fc.vbat_filtered : vbat_v;
        s_telem_snap.attitude    = s_fc.attitude;
        s_telem_snap.motors      = motors;
        s_telem_snap.loop_us     = exec_us;
        s_telem_snap.calibrating = controller_is_calibrating(&s_fc);
        s_telem_snap.imu_ok      = imu_runtime_ok;
        s_telem_snap.gyro_bias   = s_fc.gyro_bias;
        portEXIT_CRITICAL(&s_telem_mux);
    }
}

// ==================== Core0: 20Hz BLE 遥测推送 + 状态灯 + 串口调试任务 ====================
static void vCommTask(void *pvParameters) {
    (void)pvParameters;

    TickType_t last_wake     = xTaskGetTickCount();
    bool       logged_cal    = false;
    uint32_t   diag_div      = 0U;

    for (;;) {
        SharedTelem_t snap;
        portENTER_CRITICAL(&s_telem_mux);
        snap = s_telem_snap;
        portEXIT_CRITICAL(&s_telem_mux);

        BleCommStats_t ble_stats;
        ble_comm_get_stats(&ble_stats);

        // 1. 刷新板载 WS2812 五态安全/校准灯语 (E-03 可观测性)
        status_led_update(snap.state, snap.calibrating, ble_stats.connected, snap.imu_ok);

        // 2. 通过 BLE Notify 下发 19 字节遥测帧
        ble_comm_send_telemetry(snap.state, snap.vbat_v, snap.attitude, snap.motors, snap.loop_us);

        // 3. 首次完成开机陀螺仪静止校准时串口提示
        if (!logged_cal && snap.imu_ok && !snap.calibrating) {
            logged_cal = true;
            Serial.printf("[CAL] Gyro calibration OK! Bias FRD (rad/s): X=%.4f Y=%.4f Z=%.4f\n",
                          snap.gyro_bias.x, snap.gyro_bias.y, snap.gyro_bias.z);
        }

        // 4. 每 2 秒 (40 拍 @ 20Hz) 串口打印一次健康状态，方便 USB 台架联调
        if (++diag_div >= 40U) {
            diag_div = 0U;
            if (!snap.imu_ok) {
                Serial.println("[ERR] MPU6050 disconnected/fault on I2C (GPIO8 SDA / GPIO9 SCL)!");
            } else {
                Serial.printf("[STAT] st=%u cal=%d vbat=%.2fV rpy=(%.1f, %.1f, %.1f) loop=%uus ble=%d rx=%lu/%lu\n",
                              (unsigned)snap.state,
                              snap.calibrating ? 1 : 0,
                              snap.vbat_v,
                              snap.attitude.roll  * RAD_TO_DEG,
                              snap.attitude.pitch * RAD_TO_DEG,
                              snap.attitude.yaw   * RAD_TO_DEG,
                              (unsigned)snap.loop_us,
                              ble_stats.connected ? 1 : 0,
                              (unsigned long)ble_stats.rx_ok_count,
                              (unsigned long)ble_stats.rx_err_count);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEM_TASK_PERIOD_MS));
    }
}

void setup() {
    // 1. 第一时间安全锁死四路电机 PWM（必须在所有耗时初始化之前执行）
    motor_out_init();
    motor_out_stop_all();

    Serial.begin(SERIAL_BAUD_RATE);
    delay(200);
    Serial.println("\n================ Lightweight UAV Boot (ESP32-S3-N16R8) ================");

    // 2. 初始化板载状态灯与电池电压 ADC
    status_led_init();
    vbat_init();

    // 3. 初始化飞控算法核心状态机
    controller_init(&s_fc);

    // 4. 初始化 400kHz I2C 总线与 MPU6050（由 setup() + Core1 独占访问，杜绝双核总线冲突）
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);

    s_imu_boot_ok = mpu6050_init(Wire, MPU6050_I2C_ADDR);
    if (s_imu_boot_ok) {
        Serial.println("[INIT] MPU6050 OK (DLPF=94Hz, Gyro=+-2000dps, Accel=+-8g)");
    } else {
        Serial.println("[INIT] ERROR: MPU6050 probe failed! Motors locked.");
    }

    // 5. 启动低延迟 BLE GATT 服务端
    ble_comm_init(UAV_BLE_DEVICE_NAME);
    Serial.printf("[INIT] BLE GATT advertising as \"%s\"\n", UAV_BLE_DEVICE_NAME);

    // 6. 创建 Core1 实时控制任务 + Core0 遥测状态灯任务
    xTaskCreatePinnedToCore(
        vControlTask,
        "vControlTask",
        4096,
        nullptr,
        5,
        &s_ctrl_task,
        UAV_CORE_CTRL
    );

    xTaskCreatePinnedToCore(
        vCommTask,
        "vCommTask",
        4096,
        nullptr,
        2,
        nullptr,
        UAV_CORE_COMM
    );

    // 7. 启动 400Hz (2500 us) 高精度周期硬件定时器触发 Core1
    esp_timer_create_args_t timer_args = {};
    timer_args.callback        = &ctrl_timer_cb;
    timer_args.arg             = nullptr;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name            = "ctrl_400hz";
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_ctrl_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_ctrl_timer, CTRL_LOOP_PERIOD_US));

    Serial.println("[INIT] Dual-core FreeRTOS tasks & 400Hz timer started.");
}

void loop() {
    // 核心业务已分别运行在 Core1 (400Hz) 与 Core0 (20Hz)，主循环休眠释放资源
    vTaskDelay(pdMS_TO_TICKS(1000));
}
