/*
    "protocol.h" —— 飞控与地面手柄（Web / App）之间的二进制通信协议契约

    【核心设计原则（对齐 WORKPLAN P3-8 与低延迟 BLE 要求）】：
    1. 紧凑定长二进制（拒绝 JSON 字符串解析）：
       - 遥控下行帧 BleControlPacket_t 仅 13 字节；
       - 遥测上行帧 BleTelemetryPacket_t 仅 19 字节；
       - 均小于 BLE 默认最小 ATT 有效载荷（20 字节），无需协商大 MTU 即可单包直发，零分包延迟。

    2. 自然字节对齐（Natural 2-Byte Alignment）：
       - 帧内所有 uint16_t / int16_t 字段均严格排布在偶数偏移地址（Offset 4, 6, 8, 10, 16），
         兼顾紧凑性与 CPU 访存效率，前端 DataView 按偏移读写清晰直观。

    3. 四重安全校验（越界必拒）：
       - 校验长度 -> 校验双字节帧头 -> 校验 CRC-8 -> 校验各通道量程边界（如 throttle 必须在 0~1000）。
       - 任何一环不通过直接丢弃该帧，绝不污染飞控当前的 ControlCommand_t，并保持失联计时器继续累加。
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "../flight/types.h"
#include "../flight/safety.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// 1. BLE GATT 服务与特征值 UUID 定义（飞控端与前端 controller 严格共用）
// ============================================================================
#define UAV_BLE_DEVICE_NAME        "UAV-ESP32S3"
#define UAV_BLE_SERVICE_UUID       "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
// 下行控制特征值（手柄 -> 飞控）：必须使用 Write Without Response 实现 50Hz 低延迟无阻塞传输
#define UAV_BLE_CHAR_CTRL_UUID     "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
// 上行遥测特征值（飞控 -> 手柄）：使用 Notify 推送姿态、电压、安全状态与四路电机出力
#define UAV_BLE_CHAR_TELEM_UUID    "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// ============================================================================
// 2. 协议帧常量与物理量映射满量程定义
// ============================================================================
#define PROTOCOL_CTRL_HEADER_0     0xAAU
#define PROTOCOL_CTRL_HEADER_1     0x55U
#define PROTOCOL_TELEM_HEADER_0    0x55U
#define PROTOCOL_TELEM_HEADER_1    0xAAU

#define PROTOCOL_CTRL_FRAME_SIZE   13U     // 遥控帧固定 13 字节
#define PROTOCOL_TELEM_FRAME_SIZE  19U     // 遥测帧固定 19 字节

// 摇杆数字量量程约定
#define PROTOCOL_THROTTLE_MIN      0U
#define PROTOCOL_THROTTLE_MAX      1000U   // 0 ~ 1000 对应油门 0.0 ~ 1.0
#define PROTOCOL_AXIS_MIN         (-1000)
#define PROTOCOL_AXIS_MAX          1000    // -1000 ~ +1000 对应满杆输入 -1.0 ~ +1.0

// 满杆物理量上限（阶段 1 手动自稳飞行安全倾角与偏航角速度上限）
#define PROTOCOL_MAX_TILT_DEG      25.0f   // 满杆最大目标倾角：±25°（新手友好，防大倾角失速掉高）
#define PROTOCOL_MAX_TILT_RAD      (PROTOCOL_MAX_TILT_DEG * (float)M_PI / 180.0f)
#define PROTOCOL_MAX_YAW_RATE_DPS  180.0f  // 满杆最大偏航角速度：±180°/s
#define PROTOCOL_MAX_YAW_RATE_RAD  (PROTOCOL_MAX_YAW_RATE_DPS * (float)M_PI / 180.0f)

// 控制帧 flags 位掩码定义
#define PROTOCOL_FLAG_ARMED        (1U << 0) // bit0: 1=请求解锁/保持解锁, 0=上锁
#define PROTOCOL_FLAG_KILL         (1U << 1) // bit1: 1=紧急拍停(最高优先级强制上锁清零油门)
#define PROTOCOL_FLAG_HALF_RATE    (1U << 2) // bit2: 1=新手小舵量模式(倾角折半为 ±12.5°), 0=标准模式

// ============================================================================
// 3. 帧结构体定义（1 字节紧凑对齐，小端序 Little-Endian）
// ============================================================================
#pragma pack(push, 1)

// 【下行控制帧】手柄 -> 飞控（共 13 字节，50Hz 发送）
typedef struct {
    uint8_t  header[2];       // [0..1]  固定帧头：0xAA, 0x55
    uint8_t  seq;             // [2]     包序号：0~255 循环递增（用于丢包率统计）
    uint8_t  flags;           // [3]     控制标志位（见 PROTOCOL_FLAG_*）
    uint16_t throttle;        // [4..5]  油门数字量：0 ~ 1000 (映射 0.0f ~ 1.0f)
    int16_t  roll;            // [6..7]  横滚数字量：-1000 ~ +1000 (右倾为正)
    int16_t  pitch;           // [8..9]  俯仰数字量：-1000 ~ +1000 (抬头/后拉或前推按约定映射)
    int16_t  yaw;             // [10..11] 偏航数字量：-1000 ~ +1000 (顺时针/右转为正)
    uint8_t  crc8;            // [12]    前 12 字节的 CRC-8 校验值
} BleControlPacket_t;

// 【上行遥测帧】飞控 -> 手柄（共 19 字节，10~20Hz 发送）
typedef struct {
    uint8_t  header[2];       // [0..1]  固定帧头：0x55, 0xAA
    uint8_t  seq;             // [2]     遥测包序号：0~255 循环递增
    uint8_t  state;           // [3]     飞控安全状态 FlightState_t (0:LOCKED .. 4:FAILSAFE)
    uint16_t vbat_mv;         // [4..5]  电池电压（毫伏 mV，例如 3850 代表 3.85V）
    int16_t  roll_deg10;      // [6..7]  当前横滚角（0.1° 精度，例如 125 代表 12.5°）
    int16_t  pitch_deg10;     // [8..9]  当前俯仰角（0.1° 精度）
    int16_t  yaw_deg10;       // [10..11] 当前航向角（0.1° 精度，-1800 ~ +1800）
    uint8_t  motor_pct[4];    // [12..15] M1~M4 电机当前输出百分比（0 ~ 100%）
    uint16_t loop_us;         // [16..17] Core1 控制环单拍耗时（微秒 us，监控算力余量）
    uint8_t  crc8;            // [18]    前 18 字节的 CRC-8 校验值
} BleTelemetryPacket_t;

#pragma pack(pop)

// 编译期静态断言：确保结构体字节大小在任何编译器上严格等于 13 和 19 字节
#if defined(__cplusplus)
static_assert(sizeof(BleControlPacket_t) == PROTOCOL_CTRL_FRAME_SIZE, "BleControlPacket_t size must be 13 bytes");
static_assert(sizeof(BleTelemetryPacket_t) == PROTOCOL_TELEM_FRAME_SIZE, "BleTelemetryPacket_t size must be 19 bytes");
#else
_Static_assert(sizeof(BleControlPacket_t) == PROTOCOL_CTRL_FRAME_SIZE, "BleControlPacket_t size must be 13 bytes");
_Static_assert(sizeof(BleTelemetryPacket_t) == PROTOCOL_TELEM_FRAME_SIZE, "BleTelemetryPacket_t size must be 19 bytes");
#endif

// 解析状态返回码（便于调试定位丢包原因）
typedef enum {
    PROTOCOL_OK = 0,          // 解析成功
    PROTOCOL_ERR_NULL,        // 空指针
    PROTOCOL_ERR_LENGTH,      // 帧长度不匹配
    PROTOCOL_ERR_HEADER,      // 帧头字节错误
    PROTOCOL_ERR_CRC,         // CRC8 校验失败（空中射频ビット翻转）
    PROTOCOL_ERR_BOUNDS       // 字段数值越界（防畸形包注入）
} ProtocolStatus_t;

// ============================================================================
// 4. CRC-8 校验算法（多项式 0x07：x^8 + x^2 + x + 1，初值 0x00）
// ============================================================================
static inline uint8_t protocol_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00U;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

// ============================================================================
// 5. 控制帧打包与解析函数
// ============================================================================

// 将原始字节流解析并校验为飞控内部的 ControlCommand_t
// 注意：若返回非 PROTOCOL_OK，*out_cmd 不会被修改，确保异常帧绝不污染控制环
static inline ProtocolStatus_t protocol_parse_control(const uint8_t *raw_buf,
                                                      size_t len,
                                                      ControlCommand_t *out_cmd,
                                                      uint8_t *out_seq) {
    if (raw_buf == NULL || out_cmd == NULL) {
        return PROTOCOL_ERR_NULL;
    }
    if (len != PROTOCOL_CTRL_FRAME_SIZE) {
        return PROTOCOL_ERR_LENGTH;
    }

    // 1. 校验帧头 0xAA 0x55
    if (raw_buf[0] != PROTOCOL_CTRL_HEADER_0 || raw_buf[1] != PROTOCOL_CTRL_HEADER_1) {
        return PROTOCOL_ERR_HEADER;
    }

    // 2. 校验前 12 字节的 CRC-8
    uint8_t expected_crc = protocol_crc8(raw_buf, PROTOCOL_CTRL_FRAME_SIZE - 1U);
    if (raw_buf[PROTOCOL_CTRL_FRAME_SIZE - 1U] != expected_crc) {
        return PROTOCOL_ERR_CRC;
    }

    // 3. 按字节拷贝至结构体（ESP32-S3 与 x86 均为小端序）
    BleControlPacket_t pkt;
    memcpy(&pkt, raw_buf, sizeof(BleControlPacket_t));

    // 4. 严格边界检查（越界被拒，符合 WORKPLAN P3-8 验收标准）
    if (pkt.throttle > PROTOCOL_THROTTLE_MAX) {
        return PROTOCOL_ERR_BOUNDS;
    }
    if (pkt.roll  < PROTOCOL_AXIS_MIN || pkt.roll  > PROTOCOL_AXIS_MAX ||
        pkt.pitch < PROTOCOL_AXIS_MIN || pkt.pitch > PROTOCOL_AXIS_MAX ||
        pkt.yaw   < PROTOCOL_AXIS_MIN || pkt.yaw   > PROTOCOL_AXIS_MAX) {
        return PROTOCOL_ERR_BOUNDS;
    }

    // 5. 提取控制标志与舵量缩放系数（若开启新手小舵量模式，最大倾角减半）
    bool is_kill      = ((pkt.flags & PROTOCOL_FLAG_KILL) != 0U);
    bool is_armed     = ((pkt.flags & PROTOCOL_FLAG_ARMED) != 0U) && (!is_kill);
    bool is_half_rate = ((pkt.flags & PROTOCOL_FLAG_HALF_RATE) != 0U);
    float tilt_scale  = is_half_rate ? (PROTOCOL_MAX_TILT_RAD * 0.5f) : PROTOCOL_MAX_TILT_RAD;

    // 6. 转换为飞控标准物理量结构体 ControlCommand_t
    if (is_kill || !is_armed) {
        out_cmd->armed    = false;
        out_cmd->throttle = 0.0f;
    } else {
        out_cmd->armed    = true;
        out_cmd->throttle = (float)pkt.throttle / (float)PROTOCOL_THROTTLE_MAX;
    }

    out_cmd->roll_target     = ((float)pkt.roll  / (float)PROTOCOL_AXIS_MAX) * tilt_scale;
    out_cmd->pitch_target    = ((float)pkt.pitch / (float)PROTOCOL_AXIS_MAX) * tilt_scale;
    out_cmd->yaw_rate_target = ((float)pkt.yaw   / (float)PROTOCOL_AXIS_MAX) * PROTOCOL_MAX_YAW_RATE_RAD;

    if (out_seq != NULL) {
        *out_seq = pkt.seq;
    }
    return PROTOCOL_OK;
}

// 辅助打包函数（供桌面单测或模拟发射端使用）：将 ControlCommand_t 编码为 13 字节控制帧
static inline void protocol_pack_control(const ControlCommand_t *cmd,
                                         uint8_t seq,
                                         uint8_t extra_flags,
                                         BleControlPacket_t *out_pkt) {
    memset(out_pkt, 0, sizeof(BleControlPacket_t));
    out_pkt->header[0] = PROTOCOL_CTRL_HEADER_0;
    out_pkt->header[1] = PROTOCOL_CTRL_HEADER_1;
    out_pkt->seq       = seq;

    uint8_t flags = extra_flags & ~(uint8_t)PROTOCOL_FLAG_ARMED;
    if (cmd->armed) {
        flags |= PROTOCOL_FLAG_ARMED;
    }
    out_pkt->flags = flags;

    // 限幅并量化油门 0.0~1.0 -> 0~1000
    float thr = cmd->throttle;
    if (thr < 0.0f) thr = 0.0f;
    if (thr > 1.0f) thr = 1.0f;
    out_pkt->throttle = (uint16_t)(thr * (float)PROTOCOL_THROTTLE_MAX + 0.5f);

    // 限幅并量化三轴目标
    float tilt_scale = ((flags & PROTOCOL_FLAG_HALF_RATE) != 0U)
                       ? (PROTOCOL_MAX_TILT_RAD * 0.5f)
                       : PROTOCOL_MAX_TILT_RAD;

    float r_norm = cmd->roll_target / tilt_scale;
    if (r_norm < -1.0f) r_norm = -1.0f;
    if (r_norm >  1.0f) r_norm =  1.0f;
    out_pkt->roll = (int16_t)roundf(r_norm * (float)PROTOCOL_AXIS_MAX);

    float p_norm = cmd->pitch_target / tilt_scale;
    if (p_norm < -1.0f) p_norm = -1.0f;
    if (p_norm >  1.0f) p_norm =  1.0f;
    out_pkt->pitch = (int16_t)roundf(p_norm * (float)PROTOCOL_AXIS_MAX);

    float y_norm = cmd->yaw_rate_target / PROTOCOL_MAX_YAW_RATE_RAD;
    if (y_norm < -1.0f) y_norm = -1.0f;
    if (y_norm >  1.0f) y_norm =  1.0f;
    out_pkt->yaw = (int16_t)roundf(y_norm * (float)PROTOCOL_AXIS_MAX);

    out_pkt->crc8 = protocol_crc8((const uint8_t *)out_pkt, PROTOCOL_CTRL_FRAME_SIZE - 1U);
}

// ============================================================================
// 6. 遥测帧打包函数（飞控 -> 手柄）
// ============================================================================
static inline int16_t protocol_clamp_i16(float val) {
    if (val >  32767.0f) return  32767;
    if (val < -32768.0f) return -32768;
    return (int16_t)roundf(val);
}

static inline uint8_t protocol_motor_to_pct(float m) {
    if (m <= 0.0f) return 0U;
    if (m >= 1.0f) return 100U;
    return (uint8_t)(m * 100.0f + 0.5f);
}

// 将飞控当前姿态、电池电压、安全状态、四路电机输出打包为 19 字节遥测帧
static inline void protocol_pack_telemetry(uint8_t seq,
                                           FlightState_t state,
                                           float vbat_v,
                                           EulerAngle_t att_rad,
                                           MotorOutput_t motors,
                                           uint16_t loop_us,
                                           BleTelemetryPacket_t *out_pkt) {
    memset(out_pkt, 0, sizeof(BleTelemetryPacket_t));
    out_pkt->header[0] = PROTOCOL_TELEM_HEADER_0;
    out_pkt->header[1] = PROTOCOL_TELEM_HEADER_1;
    out_pkt->seq       = seq;
    out_pkt->state     = (uint8_t)state;

    // 电池电压 V -> mV
    if (vbat_v < 0.0f) vbat_v = 0.0f;
    if (vbat_v > 65.5f) vbat_v = 65.5f;
    out_pkt->vbat_mv = (uint16_t)(vbat_v * 1000.0f + 0.5f);

    // 姿态角 rad -> 0.1 deg
    const float rad2deg10 = (180.0f / (float)M_PI) * 10.0f;
    out_pkt->roll_deg10  = protocol_clamp_i16(att_rad.roll  * rad2deg10);
    out_pkt->pitch_deg10 = protocol_clamp_i16(att_rad.pitch * rad2deg10);
    out_pkt->yaw_deg10   = protocol_clamp_i16(att_rad.yaw   * rad2deg10);

    // 四路电机 0.0~1.0 -> 0~100%
    out_pkt->motor_pct[0] = protocol_motor_to_pct(motors.m1);
    out_pkt->motor_pct[1] = protocol_motor_to_pct(motors.m2);
    out_pkt->motor_pct[2] = protocol_motor_to_pct(motors.m3);
    out_pkt->motor_pct[3] = protocol_motor_to_pct(motors.m4);

    out_pkt->loop_us = loop_us;
    out_pkt->crc8    = protocol_crc8((const uint8_t *)out_pkt, PROTOCOL_TELEM_FRAME_SIZE - 1U);
}
