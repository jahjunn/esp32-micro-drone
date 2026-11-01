/**
 * protocol.ts —— 与固件端 `firmware/src/comm/protocol.h` 严格字节级对齐的协议与摇杆数学模块
 *
 * 【字节布局完全一致】：
 * - 下行控制帧 BleControlPacket_t (13 Bytes, Little-Endian)
 *   [0..1]  0xAA, 0x55
 *   [2]     seq (0~255)
 *   [3]     flags (bit0: armed, bit1: kill, bit2: half_rate)
 *   [4..5]  throttle (uint16, 0 ~ 1000)
 *   [6..7]  roll     (int16, -1000 ~ +1000)
 *   [8..9]  pitch    (int16, -1000 ~ +1000)
 *   [10..11] yaw     (int16, -1000 ~ +1000)
 *   [12]    crc8     (poly=0x07, init=0x00)
 *
 * - 上行遥测帧 BleTelemetryPacket_t (19 Bytes, Little-Endian)
 *   [0..1]  0x55, 0xAA
 *   [2]     seq (0~255)
 *   [3]     state (FlightState_t: 0..4)
 *   [4..5]  vbat_mv (uint16)
 *   [6..7]  roll_deg10 (int16, 0.1 deg)
 *   [8..9]  pitch_deg10 (int16, 0.1 deg)
 *   [10..11] yaw_deg10 (int16, 0.1 deg)
 *   [12..15] motor_pct[4] (uint8 x 4, 0~100%)
 *   [16..17] loop_us (uint16)
 *   [18]    crc8
 */

export const UAV_BLE_DEVICE_NAME     = 'UAV-ESP32S3';
export const UAV_BLE_SERVICE_UUID    = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
export const UAV_BLE_CHAR_CTRL_UUID  = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
export const UAV_BLE_CHAR_TELEM_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';

export const PROTOCOL_CTRL_FRAME_SIZE  = 13;
export const PROTOCOL_TELEM_FRAME_SIZE = 19;

export const PROTOCOL_FLAG_ARMED     = 1 << 0; // 0x01
export const PROTOCOL_FLAG_KILL      = 1 << 1; // 0x02
export const PROTOCOL_FLAG_HALF_RATE = 1 << 2; // 0x04

// 对齐 firmware/src/flight/safety.h 的安全阈值
export const SAFETY_ARM_MAX_THROTTLE = 0.05;   // 解锁允许的最大油门：5%
export const SAFETY_VBAT_WARN_V      = 3.50;   // 一级低压告警 & 地面最低解锁电压：3.50V
export const SAFETY_VBAT_LAND_V      = 3.30;   // 二级严重低压强制缓降电压：3.30V

export const FlightState = {
  LOCKED:      0,
  ARMED:       1,
  LOWBAT_WARN: 2,
  LOWBAT_LAND: 3,
  FAILSAFE:    4,
} as const;

export type FlightState = (typeof FlightState)[keyof typeof FlightState];

export const FLIGHT_STATE_LABELS: Record<FlightState, string> = {
  [FlightState.LOCKED]:      'LOCKED (已上锁)',
  [FlightState.ARMED]:       'ARMED (正常飞行)',
  [FlightState.LOWBAT_WARN]: 'LOWBAT <3.5V (低压请返航)',
  [FlightState.LOWBAT_LAND]: 'LANDING <3.3V (强制缓降)',
  [FlightState.FAILSAFE]:    'FAILSAFE (故障急停锁死)',
};

export interface ControlInputState {
  throttle: number;  // 0.0 ~ 1.0
  roll: number;      // -1.0 ~ +1.0 (已含死区、Expo 与微调 Trim)
  pitch: number;     // -1.0 ~ +1.0
  yaw: number;       // -1.0 ~ +1.0
  armed: boolean;
  kill: boolean;
  halfRate: boolean; // true = 新手挡 (±12.5°), false = 标准挡 (±25°)
}

export interface TelemetryState {
  seq: number;
  state: FlightState;
  vbatV: number;          // 电池电压 (V)
  rollDeg: number;        // 横滚角 (deg)
  pitchDeg: number;       // 俯仰角 (deg)
  yawDeg: number;         // 航向角 (deg)
  motorPct: [number, number, number, number]; // M1~M4 (0~100%)
  loopUs: number;         // Core1 控制环耗时 (us)
}

/**
 * CRC-8 校验（多项式 0x07: x^8 + x^2 + x + 1, 初值 0x00）
 * 与 firmware/src/comm/protocol.h 中的 protocol_crc8() 完全一致
 */
export function protocolCrc8(data: Uint8Array, len: number = data.length): number {
  let crc = 0x00;
  for (let i = 0; i < len; i++) {
    crc ^= data[i];
    for (let bit = 0; bit < 8; bit++) {
      if ((crc & 0x80) !== 0) {
        crc = ((crc << 1) ^ 0x07) & 0xff;
      } else {
        crc = (crc << 1) & 0xff;
      }
    }
  }
  return crc;
}

/**
 * 专业飞控摇杆曲线：中心死区过滤 + 三次方 Expo 曲线
 * @param raw      原始摇杆输入 (-1.0 ~ +1.0)
 * @param deadzone 中心死区半径 (默认 0.04 即 4%)
 * @param expo     指数柔和系数 (默认 0.35，让中心微调极其细腻，满杆仍保持 100% 输出)
 */
export function applyDeadzoneAndExpo(raw: number, deadzone = 0.04, expo = 0.35): number {
  const clamped = Math.max(-1.0, Math.min(1.0, raw));
  const absVal = Math.abs(clamped);
  if (absVal <= deadzone) {
    return 0.0;
  }
  // 扣除死区后线性归一化到 0.0 ~ 1.0
  const sign = clamped >= 0 ? 1.0 : -1.0;
  const norm = (absVal - deadzone) / (1.0 - deadzone);
  // 三次方 Expo: y = (1 - a)*x + a*x^3
  const shaped = (1.0 - expo) * norm + expo * norm * norm * norm;
  return sign * Math.max(0.0, Math.min(1.0, shaped));
}

/**
 * 将手柄当前控制状态编码为 13 字节二进制控制帧
 */
export function encodeControlFrame(input: ControlInputState, seq: number): Uint8Array {
  const buf = new Uint8Array(PROTOCOL_CTRL_FRAME_SIZE);
  const view = new DataView(buf.buffer);

  buf[0] = 0xaa;
  buf[1] = 0x55;
  buf[2] = seq & 0xff;

  let flags = 0;
  if (input.armed && !input.kill) flags |= PROTOCOL_FLAG_ARMED;
  if (input.kill) flags |= PROTOCOL_FLAG_KILL;
  if (input.halfRate) flags |= PROTOCOL_FLAG_HALF_RATE;
  buf[3] = flags;

  const thrClamped = input.kill || !input.armed ? 0.0 : Math.max(0.0, Math.min(1.0, input.throttle));
  const thrInt = Math.round(thrClamped * 1000);

  const rollInt  = Math.round(Math.max(-1.0, Math.min(1.0, input.roll)) * 1000);
  const pitchInt = Math.round(Math.max(-1.0, Math.min(1.0, input.pitch)) * 1000);
  const yawInt   = Math.round(Math.max(-1.0, Math.min(1.0, input.yaw)) * 1000);

  // 小端序写入 (Little-Endian = true)
  view.setUint16(4, thrInt, true);
  view.setInt16(6, rollInt, true);
  view.setInt16(8, pitchInt, true);
  view.setInt16(10, yawInt, true);

  buf[12] = protocolCrc8(buf, PROTOCOL_CTRL_FRAME_SIZE - 1);
  return buf;
}

/**
 * 解码并校验飞控发回的 19 字节二进制遥测帧
 */
export function decodeTelemetryFrame(raw: Uint8Array | DataView): TelemetryState | null {
  const bytes =
    raw instanceof Uint8Array
      ? raw
      : new Uint8Array(raw.buffer, raw.byteOffset, raw.byteLength);

  if (bytes.byteLength !== PROTOCOL_TELEM_FRAME_SIZE) {
    return null;
  }
  if (bytes[0] !== 0x55 || bytes[1] !== 0xaa) {
    return null;
  }

  const expectedCrc = protocolCrc8(bytes, PROTOCOL_TELEM_FRAME_SIZE - 1);
  if (bytes[PROTOCOL_TELEM_FRAME_SIZE - 1] !== expectedCrc) {
    return null;
  }

  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const seq = bytes[2];
  const state = (bytes[3] <= FlightState.FAILSAFE ? bytes[3] : FlightState.FAILSAFE) as FlightState;
  const vbatMv = view.getUint16(4, true);
  const rollDeg10 = view.getInt16(6, true);
  const pitchDeg10 = view.getInt16(8, true);
  const yawDeg10 = view.getInt16(10, true);

  const motorPct: [number, number, number, number] = [
    bytes[12],
    bytes[13],
    bytes[14],
    bytes[15],
  ];
  const loopUs = view.getUint16(16, true);

  return {
    seq,
    state,
    vbatV: vbatMv / 1000.0,
    rollDeg: rollDeg10 / 10.0,
    pitchDeg: pitchDeg10 / 10.0,
    yawDeg: yawDeg10 / 10.0,
    motorPct,
    loopUs,
  };
}

/**
 * 将二进制帧格式化为十六进制字符串（如 "AA 55 01 00 ..."），供 HUD 实时监视器使用
 */
export function formatHexFrame(bytes: Uint8Array): string {
  return Array.from(bytes)
    .map((b) => b.toString(16).toUpperCase().padStart(2, '0'))
    .join(' ');
}
