/**
 * verify_protocol.mjs —— 直接加载并验证 `src/protocol.ts` 的全套编解码、CRC-8、死区与 Expo 算法
 */
import assert from 'node:assert/strict';
import {
  FlightState,
  PROTOCOL_CTRL_FRAME_SIZE,
  PROTOCOL_TELEM_FRAME_SIZE,
  protocolCrc8,
  applyDeadzoneAndExpo,
  encodeControlFrame,
  decodeTelemetryFrame,
  formatHexFrame,
} from '../src/protocol.ts';

console.log('============================= Controller Protocol & Stick Math Test =============================');

// 1. 验证死区与三次方 Expo 曲线
assert.equal(applyDeadzoneAndExpo(0.0, 0.04, 0.35), 0.0, 'Center 0.0 must be 0.0');
assert.equal(applyDeadzoneAndExpo(0.039, 0.04, 0.35), 0.0, 'Inside 4% deadzone must be filtered to 0.0');
assert.equal(applyDeadzoneAndExpo(-0.039, 0.04, 0.35), 0.0, 'Negative deadzone must be filtered to 0.0');
assert.ok(Math.abs(applyDeadzoneAndExpo(1.0, 0.04, 0.35) - 1.0) < 1e-6, 'Full deflection +1.0 must reach +1.0');
assert.ok(Math.abs(applyDeadzoneAndExpo(-1.0, 0.04, 0.35) + 1.0) < 1e-6, 'Full deflection -1.0 must reach -1.0');
const midOut = applyDeadzoneAndExpo(0.52, 0.04, 0.35);
assert.ok(midOut > 0.3 && midOut < 0.5, 'Expo curve softens mid-stick sensitivity around hover');
console.log('[PASS] 1. Stick Deadzone (4%) & Cubic Expo (0.35) math verified');

// 2. 验证 13 字节下行控制帧编码
const ctrlFrame = encodeControlFrame(
  {
    throttle: 0.45,
    roll: 0.4,
    pitch: -0.5,
    yaw: 0.5,
    armed: true,
    kill: false,
    halfRate: false,
  },
  42
);
assert.equal(ctrlFrame.byteLength, PROTOCOL_CTRL_FRAME_SIZE, 'Control frame is 13 bytes');
assert.equal(ctrlFrame[0], 0xaa);
assert.equal(ctrlFrame[1], 0x55);
assert.equal(ctrlFrame[2], 42);
assert.equal(ctrlFrame[3], 0x01); // PROTOCOL_FLAG_ARMED
const ctrlView = new DataView(ctrlFrame.buffer);
assert.equal(ctrlView.getUint16(4, true), 450);
assert.equal(ctrlView.getInt16(6, true), 400);
assert.equal(ctrlView.getInt16(8, true), -500);
assert.equal(ctrlView.getInt16(10, true), 500);
assert.equal(ctrlFrame[12], protocolCrc8(ctrlFrame, 12));
console.log(`[PASS] 2. 13-byte Control Frame encoded: [${formatHexFrame(ctrlFrame)}]`);

// 3. 验证紧急急停 (KILL) 强制覆盖油门为 0 且清除 ARMED 位
const killFrame = encodeControlFrame(
  {
    throttle: 0.95,
    roll: 0.2,
    pitch: 0.1,
    yaw: 0.0,
    armed: true,
    kill: true,
    halfRate: false,
  },
  43
);
const killView = new DataView(killFrame.buffer);
assert.equal(killFrame[3] & 0x01, 0, 'Kill flag must clear ARMED bit');
assert.equal(killFrame[3] & 0x02, 0x02, 'Kill flag bit1 set');
assert.equal(killView.getUint16(4, true), 0, 'Kill flag must force throttle to 0');
console.log('[PASS] 3. Kill switch hardware-level override verified');

// 4. 验证 19 字节上行遥测帧解码与 CRC 破坏拦截
const telemBuf = new Uint8Array(PROTOCOL_TELEM_FRAME_SIZE);
const tView = new DataView(telemBuf.buffer);
telemBuf[0] = 0x55;
telemBuf[1] = 0xaa;
telemBuf[2] = 7;
telemBuf[3] = FlightState.ARMED;
tView.setUint16(4, 3850, true); // 3.85V
tView.setInt16(6, 55, true);    // +5.5 deg
tView.setInt16(8, -32, true);   // -3.2 deg
tView.setInt16(10, 1200, true); // +120.0 deg
telemBuf[12] = 42;
telemBuf[13] = 45;
telemBuf[14] = 41;
telemBuf[15] = 44;
tView.setUint16(16, 1120, true); // 1120 us
telemBuf[18] = protocolCrc8(telemBuf, 18);

const parsed = decodeTelemetryFrame(telemBuf);
assert.ok(parsed !== null, 'Valid telemetry frame must decode');
assert.equal(parsed.state, FlightState.ARMED);
assert.equal(parsed.vbatV, 3.85);
assert.equal(parsed.rollDeg, 5.5);
assert.equal(parsed.pitchDeg, -3.2);
assert.equal(parsed.yawDeg, 120.0);
assert.deepEqual(parsed.motorPct, [42, 45, 41, 44]);
assert.equal(parsed.loopUs, 1120);

// 制造 1 bit 翻转，验证解码器拒绝损坏的遥测包
telemBuf[6] ^= 0x01;
assert.equal(decodeTelemetryFrame(telemBuf), null, 'Corrupted telemetry frame must return null');
console.log('[PASS] 4. 19-byte Telemetry Frame decode & CRC-8 integrity check verified');
console.log('=================================================================================================');
