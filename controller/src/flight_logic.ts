/**
 * flight_logic.ts —— 地面站核心飞控控制律、坐标系转换、安全状态守卫与高保真仿真逻辑（零 DOM 依赖）
 *
 * 设计原则：
 * 1. 所有关键安全逻辑与物理映射均剥离 DOM / Web API，使桌面 Node.js 单测可直接回归验证 100% 生产代码；
 * 2. 兼容 Node 24 原生 stripTypeScriptTypes（不使用 enum / parameter properties）与 TypeScript strict 模式；
 * 3. 与固件端 `types.h` / `mixer.h` / `safety.h` / `controller.h` / `protocol.h` 保持严格物理单位与符号一致。
 */

import {
  type ControlInputState,
  FlightState,
  SAFETY_ARM_MAX_THROTTLE,
  SAFETY_VBAT_WARN_V,
  SAFETY_VBAT_LAND_V,
  applyDeadzoneAndExpo,
} from './protocol.ts';

// ==================== 全仓对齐的控制与安全常量 ====================
export const CTRL_TILT_STANDARD_DEG = 25.0;
export const CTRL_TILT_HALF_DEG = 12.5;
export const CTRL_TRIM_MAX_DEG = 5.0;
export const CTRL_TRIM_STEP_DEG = 0.2;
export const MOTOR_IDLE_DEFAULT = 0.12;

export const SAFETY_MAX_TILT_RAD = (60.0 * Math.PI) / 180.0;
export const SAFETY_TILT_RECOVER_OK_RAD = (45.0 * Math.PI) / 180.0;
export const SAFETY_EXTREME_TILT_RAD = (110.0 * Math.PI) / 180.0;
export const SAFETY_TILT_RECOVER_MAX_THROTTLE = 0.45;
export const SAFETY_LAND_MAX_THROTTLE = 0.25;

// 仿真引擎运行在 50Hz (20ms/拍)，对应固件 400Hz 下的 0.8 秒宽限期 (320 * 2.5ms = 800ms -> 40 拍 * 20ms)
export const SIM_LOOP_HZ = 50;
export const SIM_TILT_GRACE_TICKS = 40;

export function clamp(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return min;
  return Math.max(min, Math.min(max, value));
}

// ==================== 1. Mode-2 摇杆输入 -> FRD 协议指令转换 ====================
export interface RawStickState {
  throttle: number;     // 油门：0.0 ~ 1.0
  yawRight: number;     // 偏航：-1.0(左转) ~ +1.0(右转顺时针)
  rollRight: number;    // 横滚：-1.0(左倾) ~ +1.0(右倾)
  pitchForward: number; // 俯仰人机习惯：+1.0(右杆前推/前飞) ~ -1.0(右杆后拉/后退)
}

export interface PilotFlightConfig {
  armed: boolean;
  kill: boolean;
  halfRate: boolean;
  rollTrimDeg: number;  // FRD 横滚微调角 (deg)：+ 右倾，- 左倾
  pitchTrimDeg: number; // FRD 俯仰微调角 (deg)：+ 抬头(后飞)，- 低头(前飞)
}

/**
 * 将 Mode-2 人机操纵量转换为严格遵守 FRD 航空右手系的 ControlInputState：
 *   - 推右摇杆向上 (pitchForward = +1.0) -> 需机头下压前倾 -> FRD 负俯仰 (pitch = -1.0)
 *   - 拉右摇杆向下 (pitchForward = -1.0) -> 需机头抬起后仰 -> FRD 正俯仰 (pitch = +1.0)
 *   - 推右摇杆向右 (rollRight = +1.0)    -> 需右翼下压右倾 -> FRD 正横滚 (roll = +1.0)
 *   - 推左摇杆向右 (yawRight = +1.0)     -> 需俯视顺时针右转 -> FRD 正偏航 (yaw = +1.0)
 */
export function buildControlInput(
  sticks: RawStickState,
  cfg: PilotFlightConfig
): ControlInputState {
  const maxTiltDeg = cfg.halfRate ? CTRL_TILT_HALF_DEG : CTRL_TILT_STANDARD_DEG;
  const thr = clamp(sticks.throttle, 0.0, 1.0);
  const shapedYaw = applyDeadzoneAndExpo(sticks.yawRight, 0.05, 0.3);
  const shapedRoll = applyDeadzoneAndExpo(sticks.rollRight, 0.04, 0.35);
  const shapedPitchForward = applyDeadzoneAndExpo(sticks.pitchForward, 0.04, 0.35);

  const rollTrimRatio = clamp(cfg.rollTrimDeg, -CTRL_TRIM_MAX_DEG, CTRL_TRIM_MAX_DEG) / maxTiltDeg;
  const pitchTrimRatio = clamp(cfg.pitchTrimDeg, -CTRL_TRIM_MAX_DEG, CTRL_TRIM_MAX_DEG) / maxTiltDeg;

  const frdRoll = clamp(shapedRoll + rollTrimRatio, -1.0, 1.0);
  const frdPitch = clamp(-shapedPitchForward + pitchTrimRatio, -1.0, 1.0);

  return {
    throttle: thr,
    roll: frdRoll,
    pitch: frdPitch,
    yaw: shapedYaw,
    armed: cfg.armed && !cfg.kill,
    kill: cfg.kill,
    halfRate: cfg.halfRate,
  };
}

export function adjustTrimDeg(currentDeg: number, deltaDeg: number): number {
  const next = Math.round((currentDeg + deltaDeg) * 10) / 10;
  return clamp(next, -CTRL_TRIM_MAX_DEG, CTRL_TRIM_MAX_DEG);
}

export function formatRollTrimLabel(deg: number): string {
  if (Math.abs(deg) < 0.05) return '0.0°';
  return deg > 0 ? `右 +${deg.toFixed(1)}°` : `左 ${deg.toFixed(1)}°`;
}

export function formatPitchTrimLabel(deg: number): string {
  if (Math.abs(deg) < 0.05) return '0.0°';
  // FRD：负俯仰角为低头前倾，正俯仰角为抬头后仰
  return deg < 0 ? `前 ${deg.toFixed(1)}°` : `后 +${deg.toFixed(1)}°`;
}

/**
 * 手机触屏“相对位移式”摇杆数学（落指处即原点，杜绝手指一落下摇杆/油门就跳变）：
 *   - 起点归一化值 (startNorm) + 手指相对落点的位移 / 满行程像素 = 当前归一化值；
 *   - 回中轴 startNorm = 0；油门轴 startNorm = 落指时的当前油门位置，因此触摸不会改变油门，只有拖动才会；
 *   - circular = true 时做圆形矢量限幅（右摇杆俯仰+横滚），否则逐轴限幅（左摇杆油门+偏航）。
 * 坐标沿用屏幕系：Y 向下为正。
 */
export function applyStickDelta(
  startNormX: number,
  startNormY: number,
  dxPx: number,
  dyPx: number,
  maxRadiusPx: number,
  circular: boolean
): { x: number; y: number } {
  if (!(maxRadiusPx > 1e-3) || !Number.isFinite(dxPx) || !Number.isFinite(dyPx)) {
    return { x: startNormX, y: startNormY };
  }
  let x = startNormX + dxPx / maxRadiusPx;
  let y = startNormY + dyPx / maxRadiusPx;
  if (circular) {
    const r = Math.hypot(x, y);
    if (r > 1.0) {
      x /= r;
      y /= r;
    }
  } else {
    x = clamp(x, -1.0, 1.0);
    y = clamp(y, -1.0, 1.0);
  }
  return { x, y };
}

// ==================== 2. 实体手柄轴死区与防暴冲油门映射 ====================
export type GamepadThrottleMode = 'incremental' | 'direct';

/**
 * 严格遵循 AGENTS.md §7：轴数值落入 [-dz, +dz] 死区或非有限数时，显式返回 0.0
 */
export function applyAxisDeadzone(raw: number, dz = 0.05): number {
  if (!Number.isFinite(raw) || Math.abs(raw) <= dz) {
    return 0.0;
  }
  return clamp(raw, -1.0, 1.0);
}

/**
 * 标准弹簧回中手柄（Xbox / PS / 8BitDo 等）左 Y 轴增量速率油门积分：
 *   - 摇杆弹簧回中时保持当前油门（开机未解锁保持 0%，悬停松手稳在 50%），消除 50% 居中死区跳跃；
 *   - 上推缓慢柔和增油，下拉快速减油，dt 限幅 <= 50ms 防浏览器切帧卡顿暴冲。
 */
export function computeIncrementalThrottle(
  currentThrottle: number,
  upPositiveAxis: number,
  dtSec: number,
  deadzone = 0.08
): number {
  const thr = clamp(currentThrottle, 0.0, 1.0);
  const dzAxis = applyAxisDeadzone(upPositiveAxis, deadzone);
  if (dzAxis === 0.0) {
    return thr;
  }

  const clampedDt = clamp(dtSec, 0.0, 0.05);
  const mag = (Math.abs(dzAxis) - deadzone) / (1.0 - deadzone);
  const ratePerSec =
    dzAxis > 0 ? (0.18 + 0.42 * mag) * mag : (0.25 + 0.85 * mag) * mag;

  const delta = (dzAxis > 0 ? 1.0 : -1.0) * ratePerSec * clampedDt;
  return clamp(thr + delta, 0.0, 1.0);
}

export interface DirectThrottleResult {
  throttle: number;
  engaged: boolean;
}

/**
 * 航模遥控器 USB HID 绝对油门直映（带防暴冲软接管 Soft-Takeover 保护）：
 *   - 若刚切入直映模式时手柄杆不在当前油门附近（例如误插弹簧回中在 50% 的 Xbox 手柄），
 *     锁止接管直至摇杆归零（<=5%）或穿过当前油门（±6%），彻底杜绝瞬间 0% -> 50% 暴冲。
 */
export function computeDirectThrottle(
  upPositiveAxis: number,
  currentThrottle: number,
  engaged: boolean
): DirectThrottleResult {
  const thr = clamp(currentThrottle, 0.0, 1.0);
  if (!Number.isFinite(upPositiveAxis)) {
    return { throttle: thr, engaged };
  }

  const raw01 = clamp((upPositiveAxis + 1.0) * 0.5, 0.0, 1.0);
  // 2% 两端死区线性重映射至 [0.0, 1.0]，消除老式航模电位器到不了 100% 的问题且保证 0.0 -> 1.0 C0 连续无跳变，中心 0.0 精确对应 0.5
  let mapped = clamp((raw01 - 0.02) / 0.96, 0.0, 1.0);
  if (mapped < 1e-5) mapped = 0.0;
  if (mapped > 1.0 - 1e-5) mapped = 1.0;

  if (!engaged) {
    const canCapture =
      mapped <= SAFETY_ARM_MAX_THROTTLE || Math.abs(mapped - thr) <= 0.06;
    if (!canCapture) {
      return { throttle: thr, engaged: false };
    }
    return { throttle: mapped, engaged: true };
  }

  return { throttle: mapped, engaged: true };
}

// ==================== 3. E-04 断连 / 切后台 / 失焦 / 退页 安全紧急决策 ====================
export type SafetyEventReason =
  | 'visibility-hidden'
  | 'window-blur'
  | 'pagehide'
  | 'gamepad-disconnect'
  | 'ble-disconnect';

export interface LifecycleSafetyOutcome {
  zeroThrottle: boolean;
  releaseSpringAxes: boolean;
  disarm: boolean;
  latchKill: boolean;
  sendKillBurst: boolean;
  alertMessage: string | null;
}

/**
 * 根据触发安全事件的来源与触发前是否处于解锁态，输出确定性急停动作表：
 *   - 无论当前是否解锁，一律释放弹簧轴并清零油门（防止后台恢复后摇杆残留推力）；
 *   - 只要此前处于已解锁态 (wasArmed = true)，必须强制上锁、锁存 KILL、并发射 3 帧急停包。
 */
export function evaluateLifecycleSafety(
  reason: SafetyEventReason,
  wasArmed: boolean
): LifecycleSafetyOutcome {
  let alertMessage: string | null = null;

  if (wasArmed) {
    switch (reason) {
      case 'visibility-hidden':
        alertMessage = '检测到页面切入后台，已自动上锁并归零油门！';
        break;
      case 'window-blur':
        alertMessage = '检测到控制窗口失焦，已自动上锁并归零油门！';
        break;
      case 'pagehide':
        alertMessage = '页面即将离开，已触发紧急急停 (KILL)！';
        break;
      case 'gamepad-disconnect':
        alertMessage = '实体手柄已断开连接，已触发安全急停并归零油门！';
        break;
      case 'ble-disconnect':
        alertMessage = 'BLE 连接已断开，已强制上锁并归零油门！';
        break;
    }
  }

  return {
    zeroThrottle: true,
    releaseSpringAxes: true,
    disarm: true,
    latchKill: wasArmed,
    sendKillBurst: wasArmed && reason !== 'ble-disconnect',
    alertMessage,
  };
}

// ==================== 4. 离线高保真飞控与 Quad-X 混控仿真器 ====================
/**
 * 严格复刻 firmware/src/flight/mixer.h（FRD 航空右手系 + 保均值平移抗饱和）
 */
export function mixerQuadX(
  throttle: number,
  roll: number,
  pitch: number,
  yaw: number
): [number, number, number, number] {
  // M1 右前(CCW): +P -R +Y | M2 右后(CW): -P -R -Y | M3 左后(CCW): -P +R +Y | M4 左前(CW): +P +R -Y
  let m1 = throttle + pitch - roll + yaw;
  let m2 = throttle - pitch - roll - yaw;
  let m3 = throttle - pitch + roll + yaw;
  let m4 = throttle + pitch + roll - yaw;

  const avg = clamp((m1 + m2 + m3 + m4) * 0.25, 0.0, 1.0);
  const maxV = Math.max(m1, m2, m3, m4);
  const minV = Math.min(m1, m2, m3, m4);

  let scale = 1.0;
  if (maxV > 1.0 && maxV > avg) {
    scale = Math.min(scale, (1.0 - avg) / (maxV - avg));
  }
  if (minV < 0.0 && avg > minV) {
    scale = Math.min(scale, avg / (avg - minV));
  }

  if (scale < 1.0) {
    m1 = avg + (m1 - avg) * scale;
    m2 = avg + (m2 - avg) * scale;
    m3 = avg + (m3 - avg) * scale;
    m4 = avg + (m4 - avg) * scale;
  }

  return [clamp(m1, 0.0, 1.0), clamp(m2, 0.0, 1.0), clamp(m3, 0.0, 1.0), clamp(m4, 0.0, 1.0)];
}

export function mixerApplyIdle(
  motors: [number, number, number, number],
  idle = MOTOR_IDLE_DEFAULT
): [number, number, number, number] {
  const clIdle = clamp(idle, 0.0, 0.3);
  const span = 1.0 - clIdle;
  return [
    clIdle + clamp(motors[0], 0.0, 1.0) * span,
    clIdle + clamp(motors[1], 0.0, 1.0) * span,
    clIdle + clamp(motors[2], 0.0, 1.0) * span,
    clIdle + clamp(motors[3], 0.0, 1.0) * span,
  ];
}

export interface SimStepResult {
  state: FlightState;
  vbatV: number;
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
  motors: [number, number, number, number];
  highTiltTicks: number;
}

/**
 * 1:1 复刻 firmware/src/flight/safety.h + controller.h + mixer.h 行为的 50Hz 离线仿真器
 */
export class QuadSimulator {
  public state: FlightState = FlightState.LOCKED;
  public vbatV = 3.95;
  public rollDeg = 0.0;
  public pitchDeg = 0.0;
  public yawDeg = 0.0;
  public highTiltTicks = 0;

  public reset(): void {
    this.state = FlightState.LOCKED;
    this.rollDeg = 0.0;
    this.pitchDeg = 0.0;
    this.yawDeg = 0.0;
    this.highTiltTicks = 0;
  }

  public step(input: ControlInputState, holdTiltForTest = false): SimStepResult {
    const maxTiltDeg = input.halfRate ? CTRL_TILT_HALF_DEG : CTRL_TILT_STANDARD_DEG;

    const cmdValid =
      Number.isFinite(input.throttle) &&
      Number.isFinite(input.roll) &&
      Number.isFinite(input.pitch) &&
      Number.isFinite(input.yaw);

    let effThrottle = cmdValid ? clamp(input.throttle, 0.0, 1.0) : 0.0;
    let effRoll = cmdValid ? clamp(input.roll, -1.0, 1.0) : 0.0;
    let effPitch = cmdValid ? clamp(input.pitch, -1.0, 1.0) : 0.0;
    let effYaw = cmdValid ? clamp(input.yaw, -1.0, 1.0) : 0.0;

    // 1. 复刻 safety_update() 分级安全状态机
    if (!input.armed || input.kill) {
      this.state = FlightState.LOCKED;
      this.highTiltTicks = 0;
      effThrottle = 0.0;
      effRoll = 0.0;
      effPitch = 0.0;
      effYaw = 0.0;
    } else if (this.state === FlightState.FAILSAFE) {
      this.highTiltTicks = 0;
      effThrottle = 0.0;
      effRoll = 0.0;
      effPitch = 0.0;
      effYaw = 0.0;
    } else {
      const attValid = Number.isFinite(this.rollDeg) && Number.isFinite(this.pitchDeg);
      const rollRad = (this.rollDeg * Math.PI) / 180.0;
      const pitchRad = (this.pitchDeg * Math.PI) / 180.0;
      const cosTilt = attValid ? Math.cos(rollRad) * Math.cos(pitchRad) : -1.0;

      const tiltEnter = !attValid || cosTilt < Math.cos(SAFETY_MAX_TILT_RAD);
      const tiltClear = attValid && cosTilt >= Math.cos(SAFETY_TILT_RECOVER_OK_RAD);
      const tiltExtreme = !attValid || cosTilt < Math.cos(SAFETY_EXTREME_TILT_RAD);

      if (this.state === FlightState.LOCKED) {
        if (
          cmdValid &&
          !tiltEnter &&
          Number.isFinite(this.vbatV) &&
          this.vbatV >= SAFETY_VBAT_WARN_V &&
          effThrottle <= SAFETY_ARM_MAX_THROTTLE
        ) {
          this.state = FlightState.ARMED;
          this.highTiltTicks = 0;
        } else {
          this.highTiltTicks = 0;
          effThrottle = 0.0;
          effRoll = 0.0;
          effPitch = 0.0;
          effYaw = 0.0;
        }
      }

      if (
        this.state === FlightState.ARMED ||
        this.state === FlightState.LOWBAT_WARN ||
        this.state === FlightState.LOWBAT_LAND
      ) {
        if (!cmdValid || tiltExtreme) {
          this.state = FlightState.FAILSAFE;
          this.highTiltTicks = 0;
          effThrottle = 0.0;
          effRoll = 0.0;
          effPitch = 0.0;
          effYaw = 0.0;
        } else {
          if (tiltEnter || (this.highTiltTicks > 0 && !tiltClear)) {
            if (effThrottle <= SAFETY_ARM_MAX_THROTTLE) {
              this.state = FlightState.FAILSAFE;
              this.highTiltTicks = 0;
              effThrottle = 0.0;
              effRoll = 0.0;
              effPitch = 0.0;
              effYaw = 0.0;
            } else {
              this.highTiltTicks++;
              if (this.highTiltTicks >= SIM_TILT_GRACE_TICKS) {
                this.state = FlightState.FAILSAFE;
                this.highTiltTicks = 0;
                effThrottle = 0.0;
                effRoll = 0.0;
                effPitch = 0.0;
                effYaw = 0.0;
              } else {
                effRoll = 0.0;
                effPitch = 0.0;
                effYaw = 0.0;
                effThrottle = Math.min(effThrottle, SAFETY_TILT_RECOVER_MAX_THROTTLE);
              }
            }
          } else {
            this.highTiltTicks = 0;
          }

          if (this.state !== FlightState.FAILSAFE) {
            // 电池二级低压强制单向锁存（防电压回弹抽搐，对齐 AGENTS.md §6）
            if (
              this.state === FlightState.LOWBAT_LAND ||
              (Number.isFinite(this.vbatV) && this.vbatV < SAFETY_VBAT_LAND_V)
            ) {
              this.state = FlightState.LOWBAT_LAND;
              effRoll = 0.0;
              effPitch = 0.0;
              effYaw = 0.0;
              effThrottle = Math.min(effThrottle, SAFETY_LAND_MAX_THROTTLE);
            } else if (Number.isFinite(this.vbatV) && this.vbatV < SAFETY_VBAT_WARN_V) {
              this.state = FlightState.LOWBAT_WARN;
            } else {
              this.state = FlightState.ARMED;
            }
          }
        }
      }
    }

    // 2. 地面怠速静默 + Quad-X 闭环混控 + 12% 保底（对齐 controller.h + mixer.h）
    const targetRollDeg = effRoll * maxTiltDeg;
    const targetPitchDeg = effPitch * maxTiltDeg;

    let motors: [number, number, number, number] = [0.0, 0.0, 0.0, 0.0];
    if (this.state === FlightState.LOCKED || this.state === FlightState.FAILSAFE) {
      if (!holdTiltForTest && Number.isFinite(this.rollDeg) && Number.isFinite(this.pitchDeg)) {
        this.rollDeg *= 0.85;
        this.pitchDeg *= 0.85;
      }
    } else if (effThrottle <= SAFETY_ARM_MAX_THROTTLE) {
      // 地面怠速静默态：四路均匀输出 12%
      motors = [MOTOR_IDLE_DEFAULT, MOTOR_IDLE_DEFAULT, MOTOR_IDLE_DEFAULT, MOTOR_IDLE_DEFAULT];
      if (!holdTiltForTest) {
        this.rollDeg += (targetRollDeg - this.rollDeg) * 0.15;
        this.pitchDeg += (targetPitchDeg - this.pitchDeg) * 0.15;
      }
    } else {
      const normThr = clamp((effThrottle - SAFETY_ARM_MAX_THROTTLE) / (1.0 - SAFETY_ARM_MAX_THROTTLE), 0.0, 1.0);
      const mixThr = normThr * (1.0 - SAFETY_ARM_MAX_THROTTLE);

      const rTorque = clamp(effRoll * 0.10 + ((targetRollDeg - this.rollDeg) / 25.0) * 0.22, -0.40, 0.40);
      const pTorque = clamp(effPitch * 0.10 + ((targetPitchDeg - this.pitchDeg) / 25.0) * 0.22, -0.40, 0.40);
      const yTorque = clamp(effYaw * 0.15, -0.30, 0.30);

      const rawMotors = mixerQuadX(mixThr, rTorque, pTorque, yTorque);
      motors = mixerApplyIdle(rawMotors, MOTOR_IDLE_DEFAULT);

      if (!holdTiltForTest) {
        this.rollDeg += (targetRollDeg - this.rollDeg) * 0.15;
        this.pitchDeg += (targetPitchDeg - this.pitchDeg) * 0.15;
        this.yawDeg = ((this.yawDeg + effYaw * 180.0 * 0.02 + 540.0) % 360.0) - 180.0;
      }
    }

    return {
      state: this.state,
      vbatV: this.vbatV,
      rollDeg: this.rollDeg,
      pitchDeg: this.pitchDeg,
      yawDeg: this.yawDeg,
      motors,
      highTiltTicks: this.highTiltTicks,
    };
  }
}
