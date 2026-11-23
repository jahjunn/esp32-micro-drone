#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * heartbeat.h —— 控制环跨核存活心跳判定（纯 C 无 Arduino 依赖，可独立桌面单测）
 *
 * 【设计说明（F-01）】：
 *   - last_seen_ms 由 Core1 400Hz 控制环每拍末尾刷新，now_ms 由 Core0 20Hz 监控任务读取；
 *   - 基于 uint32_t 模 2^32 无符号差值天然支持 millis() 49.7 天溢出回绕；
 *   - 若因双核极窄竞态窗口导致 last_seen_ms 略超前于 now_ms（差值落入 [2^31, 2^32)），
 *     说明刚被另一核刷新，直接视为存活（false），严防下溢误触发空中停桨。
 */
static inline bool heartbeat_stale(uint32_t last_seen_ms, uint32_t now_ms, uint32_t timeout_ms) {
    uint32_t elapsed = (uint32_t)(now_ms - last_seen_ms);
    if (elapsed >= 0x80000000U) {
        return false;
    }
    return elapsed > timeout_ms;
}

// 调用方在同一跨核锁内完成判定与 PWM 写入；只有新鲜的主动上锁命令允许复位。
static inline bool heartbeat_fault_latched(bool fault, uint32_t last_seen_ms,
                                           uint32_t now_ms, uint32_t timeout_ms,
                                           bool healthy_disarm) {
    return !healthy_disarm && (fault || heartbeat_stale(last_seen_ms, now_ms, timeout_ms));
}
