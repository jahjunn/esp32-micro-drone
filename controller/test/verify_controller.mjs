/**
 * verify_controller.mjs —— 地面站前端控制律、Mode-2->FRD 符号对齐、E-04 安全保护与仿真闭环单测
 */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { stripTypeScriptTypes } from 'node:module';
import { fileURLToPath } from 'node:url';

import { FlightState, encodeControlFrame } from '../src/protocol.ts';
import {
  CTRL_TILT_STANDARD_DEG,
  MOTOR_IDLE_DEFAULT,
  QuadSimulator,
  SAFETY_TILT_RECOVER_MAX_THROTTLE,
  SIM_TILT_GRACE_TICKS,
  adjustTrimDeg,
  applyAxisDeadzone,
  applyStickDelta,
  buildControlInput,
  computeDirectThrottle,
  computeIncrementalThrottle,
  evaluateLifecycleSafety,
  formatPitchTrimLabel,
  formatRollTrimLabel,
  mixerQuadX,
} from '../src/flight_logic.ts';
import { UavBleController } from '../src/ble_adapter.ts';

console.log('============================= Ground Station Control & Safety Test =============================');

// 1. 验证全部 controller/src/*.ts 可被 Node 24 stripTypeScriptTypes 无错剥离（与 dev_server.mjs 保持兼容）
const testDir = path.dirname(fileURLToPath(import.meta.url));
const srcDir = path.resolve(testDir, '../src');
const tsFiles = fs.readdirSync(srcDir).filter((f) => f.endsWith('.ts'));
assert.ok(tsFiles.length >= 5, 'Should find all controller TS modules');
for (const file of tsFiles) {
  const src = fs.readFileSync(path.join(srcDir, file), 'utf8');
  const js = stripTypeScriptTypes(src, { mode: 'strip' });
  assert.ok(js.length > 0, `stripTypeScriptTypes succeeded for ${file}`);
}
console.log(`[PASS] 1. Node 24 TypeScript type-stripping verified across ${tsFiles.length} modules`);

// 2. 验证 Mode-2 摇杆 & Trim 微调 -> 航空 FRD 坐标系严格对齐（消除前推反向飞大坑）
const fwdInput = buildControlInput(
  { throttle: 0.55, yawRight: 0.0, rollRight: 0.0, pitchForward: 1.0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0.0, pitchTrimDeg: 0.0 }
);
assert.equal(fwdInput.pitch, -1.0, 'Mode-2 stick forward (+1.0) must command FRD nose-down pitch (-1.0)');
const fwdFrame = encodeControlFrame(fwdInput, 1);
const fwdView = new DataView(fwdFrame.buffer);
assert.equal(fwdView.getInt16(8, true), -1000, 'BLE wire frame pitch for stick-forward must be -1000');

const backInput = buildControlInput(
  { throttle: 0.55, yawRight: 0.0, rollRight: 0.0, pitchForward: -1.0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0.0, pitchTrimDeg: 0.0 }
);
assert.equal(backInput.pitch, 1.0, 'Mode-2 stick back (-1.0) must command FRD nose-up pitch (+1.0)');

// 点击 2 次 "▲ 前微调" (-0.2 deg * 2 = -0.4 deg FRD)
let pTrim = adjustTrimDeg(0.0, -0.2);
pTrim = adjustTrimDeg(pTrim, -0.2);
assert.equal(pTrim, -0.4);
assert.equal(formatPitchTrimLabel(pTrim), '前 -0.4°');
assert.equal(formatRollTrimLabel(0.4), '右 +0.4°');
const trimOnlyInput = buildControlInput(
  { throttle: 0.50, yawRight: 0.0, rollRight: 0.0, pitchForward: 0.0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0.0, pitchTrimDeg: pTrim }
);
assert.ok(
  Math.abs(trimOnlyInput.pitch - (-0.4 / CTRL_TILT_STANDARD_DEG)) < 1e-6,
  'Forward trim produces negative FRD pitch target'
);
console.log('[PASS] 2. Mode-2 -> Aerospace FRD Pitch/Roll/Yaw & Trim sign alignment verified');

// 3. 验证 E-04 全场景切后台 / 失焦 / 退页 / 手柄断连 / BLE 断连安全决策
const armedReasons = [
  'visibility-hidden',
  'window-blur',
  'pagehide',
  'gamepad-disconnect',
  'ble-disconnect',
];
for (const reason of armedReasons) {
  const out = evaluateLifecycleSafety(reason, true);
  assert.equal(out.zeroThrottle, true, `${reason} (armed) must zero throttle`);
  assert.equal(out.releaseSpringAxes, true, `${reason} (armed) must release spring axes`);
  assert.equal(out.disarm, true, `${reason} (armed) must disarm`);
  assert.equal(out.latchKill, true, `${reason} (armed) must latch kill`);
  assert.equal(out.sendKillBurst, reason !== 'ble-disconnect', `${reason} kill burst rule`);
  assert.ok(out.alertMessage && out.alertMessage.length > 0, `${reason} (armed) must emit alert`);
}
// 未解锁时 window.blur（如点击连接 BLE 弹出 Chrome 设备选框）不得误弹恐吓急停或发 KILL，但仍释放弹簧轴与归零油门
const groundBlur = evaluateLifecycleSafety('window-blur', false);
assert.equal(groundBlur.zeroThrottle, true);
assert.equal(groundBlur.releaseSpringAxes, true);
assert.equal(groundBlur.disarm, true);
assert.equal(groundBlur.latchKill, false);
assert.equal(groundBlur.sendKillBurst, false);
assert.equal(groundBlur.alertMessage, null);
console.log('[PASS] 3. E-04 background/blur/pagehide/disconnect emergency disarm & zero-throttle verified');

// 4. 验证 AGENTS.md §7 死区显式归零 + Xbox/PS 增量无 50% 跳跃 + 直映防暴冲保护
assert.equal(applyAxisDeadzone(0.049, 0.05), 0.0, 'Within 0.05 deadzone must explicitly return 0.0');
assert.equal(applyAxisDeadzone(-0.05, 0.05), 0.0, 'At -0.05 boundary must explicitly return 0.0');
assert.equal(applyAxisDeadzone(Number.NaN, 0.05), 0.0, 'NaN axis must fail-safe to 0.0');
assert.equal(applyAxisDeadzone(0.4, 0.05), 0.4, 'Outside deadzone passes value');

// Xbox 弹簧回中静止 (axis = 0.0) 不得让油门跳至 50%
assert.equal(computeIncrementalThrottle(0.0, 0.0, 0.016), 0.0, 'Centered Xbox stick holds 0% safely');
// 在 50% 悬停区平滑通过 0.49 -> 0.50 -> 0.51，无任何居中死区跳跃
let t = 0.49;
for (let i = 0; i < 4; i++) {
  t = computeIncrementalThrottle(t, 0.35, 0.02);
}
assert.ok(t > 0.495 && t < 0.55, `Incremental throttle glides smoothly across 50% hover zone (${t.toFixed(4)})`);
// 剧烈帧卡顿 dt = 5.0s 被限幅到 50ms
const stalled = computeIncrementalThrottle(0.0, 1.0, 5.0);
assert.ok(stalled < 0.05, `Large dt glitch clamped to prevent throttle spike (${stalled.toFixed(4)})`);

// 直映模式（航模控）Soft-Takeover：Xbox 摇杆卡在 50% 切入时禁止瞬跳，必须拉低 <=5% 后才能接管并直通 50% 无洞
const blockedDirect = computeDirectThrottle(0.0, 0.0, false);
assert.equal(blockedDirect.engaged, false);
assert.equal(blockedDirect.throttle, 0.0);
const engagedDirect = computeDirectThrottle(-0.96, 0.0, false);
assert.equal(engagedDirect.engaged, true);
assert.equal(engagedDirect.throttle, 0.0);
const midDirect = computeDirectThrottle(0.0, engagedDirect.throttle, engagedDirect.engaged);
assert.equal(midDirect.throttle, 0.5, 'Direct mode reaches exact 50% hover cleanly after takeover');
console.log('[PASS] 4. Gamepad §7 explicit deadzone 0.0 & zero-jump incremental/soft-takeover throttle verified');

// 5. 验证离线仿真器 Quad-X FRD 混控符号 & 12% 怠速 & E-02 分级保护 & §6 低压单向锁存
// 5a. FRD 混控公式验证
const [m1R, m2R, m3R, m4R] = mixerQuadX(0.5, 0.1, 0.0, 0.0); // 右滚 +0.1 -> 左电机 M3,M4 > 右电机 M1,M2
assert.ok(m3R > m1R + 0.15 && m4R > m2R + 0.15, 'FRD +Roll boosts M3/M4 (Left)');
const [m1P, m2P, m3P, m4P] = mixerQuadX(0.5, 0.0, -0.1, 0.0); // 前推俯仰 -0.1 -> 后电机 M2,M3 > 前电机 M1,M4
assert.ok(m2P > m1P + 0.15 && m3P > m4P + 0.15, 'FRD -Pitch (Forward) boosts M2/M3 (Rear)');

const sim = new QuadSimulator();
// 高油门拒绝解锁
let res = sim.step(buildControlInput(
  { throttle: 0.4, yawRight: 0, rollRight: 0, pitchForward: 0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
));
assert.equal(res.state, FlightState.LOCKED);
assert.deepEqual(res.motors, [0, 0, 0, 0]);

// 低油门正常解锁进入 12% 均匀地面怠速
res = sim.step(buildControlInput(
  { throttle: 0.0, yawRight: 0, rollRight: 0, pitchForward: 0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
));
assert.equal(res.state, FlightState.ARMED);
assert.deepEqual(res.motors, [MOTOR_IDLE_DEFAULT, MOTOR_IDLE_DEFAULT, MOTOR_IDLE_DEFAULT, MOTOR_IDLE_DEFAULT]);

// 前推右摇杆 (+1.0 -> FRD -1.0 -> 后侧电机 M2, M3 高于前侧 M1, M4)
res = sim.step(buildControlInput(
  { throttle: 0.55, yawRight: 0, rollRight: 0, pitchForward: 1.0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
));
assert.ok(res.motors[1] > res.motors[0] && res.motors[2] > res.motors[3], 'Forward pitch stick boosts rear M2/M3 in QuadSimulator');

// E-02：75° 阵风救机（不死机、右电机 M1/M2 大于左电机 M3/M4 回正、滞回 52° 保持计数、回 30° 清零）
sim.reset();
sim.step(buildControlInput(
  { throttle: 0.0, yawRight: 0, rollRight: 0, pitchForward: 0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
));
sim.rollDeg = 75.0;
const flyInput = buildControlInput(
  { throttle: 0.75, yawRight: 0, rollRight: 0, pitchForward: 0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
);
res = sim.step(flyInput, true);
assert.equal(res.state, FlightState.ARMED, '75 deg gust keeps flight alive for self-righting');
assert.equal(res.highTiltTicks, 1);
assert.ok(res.motors[0] > res.motors[2] && res.motors[1] > res.motors[3], 'Right motors M1/M2 boost to right +75 deg roll');
// 验证油门被限制在 45%（mixThr=0.40 映射回 [0.12, 1.0] 均值约为 0.12 + 0.40*0.88 = 0.472）
const avgMotor = (res.motors[0] + res.motors[1] + res.motors[2] + res.motors[3]) * 0.25;
const expectedRecoveryAvg = MOTOR_IDLE_DEFAULT + ((SAFETY_TILT_RECOVER_MAX_THROTTLE - 0.05) / 0.95) * 0.95 * (1.0 - MOTOR_IDLE_DEFAULT);
assert.ok(Math.abs(avgMotor - expectedRecoveryAvg) < 0.02, '75 deg gust caps average thrust to 45% recovery cap');

sim.rollDeg = 52.0; // 处于 (45°, 60°] 滞回区：不得清零计数
res = sim.step(flyInput, true);
assert.equal(res.highTiltTicks, 2, '52 deg hysteresis holds tilt timer');

sim.rollDeg = 30.0; // <= 45° 安全复位
res = sim.step(flyInput, true);
assert.equal(res.highTiltTicks, 0, '<=45 deg resets tilt timer');

// E-02：持续 >60° 达 0.8s (40 拍 @ 50Hz) 触发 FAILSAFE
sim.rollDeg = 65.0;
for (let i = 1; i < SIM_TILT_GRACE_TICKS; i++) {
  res = sim.step(flyInput, true);
  assert.equal(res.state, FlightState.ARMED);
}
res = sim.step(flyInput, true);
assert.equal(res.state, FlightState.FAILSAFE, '40 ticks (0.8s) >60 deg triggers FAILSAFE');
assert.deepEqual(res.motors, [0, 0, 0, 0]);

// E-02：120° 极端倒扣 & 低油门怠速翻倒单拍立即 FAILSAFE
sim.reset();
sim.step(buildControlInput(
  { throttle: 0.0, yawRight: 0, rollRight: 0, pitchForward: 0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
));
sim.rollDeg = 120.0;
res = sim.step(flyInput, true);
assert.equal(res.state, FlightState.FAILSAFE, '>110 deg cuts motors in 1 tick');

// AGENTS.md §6：LOWBAT_LAND (<3.30V) 必须单向锁存，即使电压回弹到 3.95V 仍不得跳回 ARMED！
sim.reset();
sim.step(buildControlInput(
  { throttle: 0.0, yawRight: 0, rollRight: 0, pitchForward: 0 },
  { armed: true, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
));
sim.vbatV = 3.20;
res = sim.step(flyInput, true);
assert.equal(res.state, FlightState.LOWBAT_LAND);
sim.vbatV = 3.95; // 电压回弹
res = sim.step(flyInput, true);
assert.equal(res.state, FlightState.LOWBAT_LAND, 'LOWBAT_LAND remains sticky even when VBAT rebounds to 3.95V');
res = sim.step(buildControlInput(
  { throttle: 0.0, yawRight: 0, rollRight: 0, pitchForward: 0 },
  { armed: false, kill: false, halfRate: false, rollTrimDeg: 0, pitchTrimDeg: 0 }
), true);
assert.equal(res.state, FlightState.LOCKED, 'Disarm resets LOWBAT_LAND latch cleanly');
console.log('[PASS] 5. QuadSimulator 1:1 parity with mixer.h, E-02 tiered tilt, and §6 sticky LOWBAT_LAND verified');

// 6. 验证 UavBleController 在无 DOM 环境下可安全运行仿真闭环并即时停机断开
let lastTelem = null;
const txFrames = [];
const ble = new UavBleController(
  () => ({ throttle: 0, roll: 0, pitch: 0, yaw: 0, armed: false, kill: false, halfRate: false }),
  {
    onModeChange: () => {},
    onTelemetry: (tVal) => { lastTelem = tVal; },
    onTxFrame: (f) => { txFrames.push(f); },
    onError: (msg) => { throw new Error(`Unexpected BLE error: ${msg}`); },
  }
);
ble.startSimulator();
await ble.sendEmergencyKillBurst();
ble.disconnect();
assert.equal(txFrames.length, 3, 'Emergency kill burst emits 3 control frames');
assert.ok(lastTelem !== null && lastTelem.state === FlightState.LOCKED, 'Kill burst updates simulator telemetry to LOCKED');
console.log('[PASS] 6. UavBleController headless lifecycle & 3-frame KILL burst verified');
// 7. 验证手机触屏“相对位移式”摇杆：手指落下不跳变油门，拖动才改变；右摇杆圆形限幅
// 油门当前在底部 (normY = +1.0 → 0%)，手指落在任何位置、位移为 0 时油门必须保持不变
let stick = applyStickDelta(0.0, 1.0, 0, 0, 100, false);
assert.deepEqual(stick, { x: 0.0, y: 1.0 }, 'Touching anywhere with zero movement must not change throttle stick');
// 向上拖 76px（满行程 2×0.38 的 100px 基准）→ normY = 0.24 → 油门 = (1-0.24)/2 = 38%
stick = applyStickDelta(0.0, 1.0, 0, -76, 100, false);
assert.ok(Math.abs((1.0 - stick.y) * 0.5 - 0.38) < 1e-9, 'Throttle follows relative drag distance');
// 已有 60% 油门（normY = -0.2）时重新落指：保持 60%，不会跳到手指绝对位置
stick = applyStickDelta(0.0, -0.2, 0, 0, 100, false);
assert.equal(stick.y, -0.2, 'Re-grab keeps existing throttle level');
// 油门轴逐轴限幅：大幅向右拖偏航不得把油门拉低
stick = applyStickDelta(0.0, -0.2, 500, 0, 100, false);
assert.deepEqual(stick, { x: 1.0, y: -0.2 }, 'Left stick clamps per-axis so yaw does not alter throttle');
// 右摇杆圆形矢量限幅：斜推 (150, -150) 的合矢量长度必须 <= 1
stick = applyStickDelta(0.0, 0.0, 150, -150, 100, true);
assert.ok(Math.abs(Math.hypot(stick.x, stick.y) - 1.0) < 1e-9, 'Right stick vector clamped to unit circle');
// 非法输入 / 零半径：保持起点，绝不产生 NaN
assert.deepEqual(applyStickDelta(0.3, -0.4, Number.NaN, 0, 100, true), { x: 0.3, y: -0.4 });
assert.deepEqual(applyStickDelta(0.3, -0.4, 10, 10, 0, true), { x: 0.3, y: -0.4 });
console.log('[PASS] 7. Mobile relative-drag stick: no jump on touch-down, per-axis/circular clamp, NaN-safe');
console.log('===============================================================================================');
