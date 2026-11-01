/*
    "ble_server.h" —— ESP32-S3 低延迟 BLE 遥控与遥测服务接口（运行于 Core0）

    【设计要点】：
    1. 零积压跨核传递：
       Core0 的 BLE 写入回调在校验通过 13 字节控制帧后，使用自旋锁（portMUX_TYPE）
       直接覆盖最新指令结构体与接收时间戳；Core1 的 400Hz 控制环读取时耗时 < 1us，
       绝不阻塞控制环，也绝不排队消费过期的历史摇杆包。

    2. 主动协商 7.5ms ~ 15ms 低延迟连接间隔：
       客户端连上瞬间，飞控主动请求 BLE 连接参数更新（min=7.5ms, max=15ms, latency=0），
       消除默认 30~50ms 间隔带来的操控粘滞感。

    3. 断链与超时双重保险：
       - 若 50Hz 遥控包断流超过 500ms，ble_comm_get_command() 返回的 cmd_age_ms > 500ms，
         由 safety_update() 自动触发 FLIGHT_STATE_FAILSAFE 急停锁死；
       - 若 BLE 物理断开连接，立即标记失联并自动重启广播，允许手柄随时重连。
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// 通信统计信息（用于串口调试或遥测监控丢包率）
typedef struct {
    uint32_t rx_ok_count;     // 成功解析的控制帧总数
    uint32_t rx_err_count;    // 校验失败或越界被拒的异常帧总数
    uint8_t  last_seq;        // 最近一次收到的包序号
    bool     connected;       // 当前是否有手柄处于 BLE 连接状态
} BleCommStats_t;

// 1. 初始化 BLE GATT 服务端并开始广播（在 setup() 中调用）
void ble_comm_init(const char *device_name);

// 2. Core1 控制环调用：获取最新一帧遥控指令及该指令距离现在的年龄（毫秒）
//    - 若从未收到过有效帧或 BLE 已断开，*out_cmd_age_ms 将返回 > SAFETY_TIMEOUT_MS (例如 99999)，
//      确保 safety_update() 严守锁定或切入 FAILSAFE。
void ble_comm_get_command(ControlCommand_t *out_cmd, uint32_t *out_cmd_age_ms);

// 3. Core0 遥测任务调用（建议 10~20Hz）：向手柄推送 19 字节姿态与电池遥测帧
bool ble_comm_send_telemetry(FlightState_t state,
                             float vbat_v,
                             EulerAngle_t att_rad,
                             MotorOutput_t motors,
                             uint16_t loop_us);

// 4. 获取当前 BLE 通信统计状态
void ble_comm_get_stats(BleCommStats_t *out_stats);

#ifdef __cplusplus
}
#endif
