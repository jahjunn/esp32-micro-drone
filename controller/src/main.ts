/**
 * main.ts —— 手柄主控制逻辑（连接虚拟双摇杆、物理手柄、安全联锁、微调 Trim 与 BLE 50Hz 引擎）
 */

import {
  type ControlInputState,
  type TelemetryState,
  FlightState,
  FLIGHT_STATE_LABELS,
  SAFETY_ARM_MAX_THROTTLE,
  SAFETY_VBAT_WARN_V,
  SAFETY_VBAT_LAND_V,
  applyDeadzoneAndExpo,
  formatHexFrame,
} from './protocol.ts';
import { UavBleController, type ConnectionMode } from './ble_adapter.ts';
import { VirtualStick, GamepadPoller } from './joystick.ts';

// ==================== 控制状态变量 ====================
let isArmed = false;
let isKillLatch = false;
let isHalfRate = false; // false = 标准 ±25°, true = 新手 ±12.5°

// 微调偏置（单位：度，每步 ±0.2°，最大 ±5.0°）
let rollTrimDeg = 0.0;
let pitchTrimDeg = 0.0;

// DOM 引用
const connDot = document.getElementById('conn-dot')!;
const connLabel = document.getElementById('conn-label')!;
const flightStatePill = document.getElementById('flight-state-pill')!;
const vbatBox = document.getElementById('vbat-box')!;
const vbatVal = document.getElementById('vbat-val')!;
const txHzVal = document.getElementById('tx-hz-val')!;
const loopUsVal = document.getElementById('loop-us-val')!;

const btnBleConnect = document.getElementById('btn-ble-connect')!;
const btnSimToggle = document.getElementById('btn-sim-toggle')!;
const btnKill = document.getElementById('btn-kill')!;
const btnZeroThr = document.getElementById('btn-zero-thr')!;
const btnArmToggle = document.getElementById('btn-arm-toggle')!;
const btnRateMode = document.getElementById('btn-rate-mode')!;

const valThr = document.getElementById('val-thr')!;
const valYaw = document.getElementById('val-yaw')!;
const valPitch = document.getElementById('val-pitch')!;
const valRoll = document.getElementById('val-roll')!;

const horizonPlate = document.getElementById('horizon-plate')!;
const attRoll = document.getElementById('att-roll')!;
const attPitch = document.getElementById('att-pitch')!;
const attYaw = document.getElementById('att-yaw')!;

const m1Bar = document.getElementById('m1-bar')!;
const m2Bar = document.getElementById('m2-bar')!;
const m3Bar = document.getElementById('m3-bar')!;
const m4Bar = document.getElementById('m4-bar')!;

const trimRollVal = document.getElementById('trim-roll-val')!;
const trimPitchVal = document.getElementById('trim-pitch-val')!;
const hexFrameView = document.getElementById('hex-frame-view')!;
const gamepadStatus = document.getElementById('gamepad-status')!;
const simToolbar = document.getElementById('sim-toolbar')!;
const alertBanner = document.getElementById('alert-banner')!;

let alertTimeoutId: number | null = null;
function showAlert(msg: string): void {
  alertBanner.textContent = msg;
  alertBanner.style.display = 'block';
  if (alertTimeoutId !== null) clearTimeout(alertTimeoutId);
  alertTimeoutId = window.setTimeout(() => {
    alertBanner.style.display = 'none';
  }, 3800);
}

// ==================== 初始化左右双虚拟摇杆 (Mode 2) ====================
const leftStick = new VirtualStick(
  document.getElementById('stick-left-zone')!,
  document.getElementById('stick-left-knob')!,
  { springX: true, springY: false, initialY: 1.0 }, // initialY = 1.0 对应底部 0% 油门
  updateStickReadouts
);

const rightStick = new VirtualStick(
  document.getElementById('stick-right-zone')!,
  document.getElementById('stick-right-knob')!,
  { springX: true, springY: true, initialY: 0.0 },
  updateStickReadouts
);

// ==================== 组合当前控制输入（含死区、Expo 与微调） ====================
function getCurrentControlInput(): ControlInputState {
  const maxTiltDeg = isHalfRate ? 12.5 : 25.0;
  const rawThr = leftStick.getY();
  const shapedYaw = applyDeadzoneAndExpo(leftStick.getX(), 0.05, 0.3);
  const shapedRoll = applyDeadzoneAndExpo(rightStick.getX(), 0.04, 0.35);
  const shapedPitch = applyDeadzoneAndExpo(rightStick.getY(), 0.04, 0.35);

  // 叠加配平微调角度
  const finalRoll = Math.max(-1.0, Math.min(1.0, shapedRoll + rollTrimDeg / maxTiltDeg));
  const finalPitch = Math.max(-1.0, Math.min(1.0, shapedPitch + pitchTrimDeg / maxTiltDeg));

  return {
    throttle: rawThr,
    roll: finalRoll,
    pitch: finalPitch,
    yaw: shapedYaw,
    armed: isArmed,
    kill: isKillLatch,
    halfRate: isHalfRate,
  };
}

function updateStickReadouts(): void {
  const input = getCurrentControlInput();
  const maxTiltDeg = isHalfRate ? 12.5 : 25.0;
  valThr.textContent = `${Math.round(input.throttle * 100)}%`;
  valYaw.textContent = `${Math.round(input.yaw * 180)}°/s`;
  valPitch.textContent = `${(input.pitch * maxTiltDeg).toFixed(1)}°`;
  valRoll.textContent = `${(input.roll * maxTiltDeg).toFixed(1)}°`;
}

// ==================== 初始化 BLE 与仿真控制器 ====================
const bleController = new UavBleController(getCurrentControlInput, {
  onModeChange: (mode: ConnectionMode, deviceName: string) => {
    connLabel.textContent = deviceName;
    connDot.className = 'status-dot';
    if (mode === 'ble-web' || mode === 'ble-native') {
      connDot.classList.add('online');
      btnBleConnect.textContent = '断开 BLE';
      simToolbar.style.display = 'none';
    } else if (mode === 'simulator') {
      connDot.classList.add('sim');
      btnBleConnect.textContent = '连接 BLE 飞控';
      simToolbar.style.display = 'flex';
    } else {
      btnBleConnect.textContent = '连接 BLE 飞控';
      simToolbar.style.display = 'none';
      // 断线自动回到锁定状态
      setArmedState(false);
    }
  },

  onTelemetry: (telem: TelemetryState) => {
    // 1. 更新安全状态徽章
    flightStatePill.textContent = FLIGHT_STATE_LABELS[telem.state];
    flightStatePill.className = 'state-pill';
    if (telem.state === FlightState.LOCKED) {
      flightStatePill.classList.add('state-locked');
    } else if (telem.state === FlightState.ARMED) {
      flightStatePill.classList.add('state-armed');
    } else if (telem.state === FlightState.LOWBAT_WARN) {
      flightStatePill.classList.add('state-warn');
    } else {
      flightStatePill.classList.add('state-failsafe');
      if (telem.state === FlightState.FAILSAFE && isArmed) {
        btnArmToggle.textContent = '故障锁死！点击上锁复位 (RESET)';
      }
    }

    // 2. 更新 1S 电池电压与分级告警色
    vbatVal.textContent = `${telem.vbatV.toFixed(2)} V`;
    vbatBox.className = 'metric-box';
    if (telem.vbatV < SAFETY_VBAT_LAND_V) {
      vbatBox.classList.add('danger');
    } else if (telem.vbatV < SAFETY_VBAT_WARN_V) {
      vbatBox.classList.add('warn');
    }

    // 3. 更新控制环耗时
    loopUsVal.textContent = `${telem.loopUs} us`;

    // 4. 更新人工地平仪与欧拉角
    attRoll.textContent = `${telem.rollDeg.toFixed(1)}°`;
    attPitch.textContent = `${telem.pitchDeg.toFixed(1)}°`;
    attYaw.textContent = `${telem.yawDeg.toFixed(1)}°`;
    const pitchPx = Math.max(-30, Math.min(30, telem.pitchDeg * 0.9));
    horizonPlate.style.transform = `rotate(${-telem.rollDeg}deg) translateY(${pitchPx}px)`;

    // 5. 更新 4 路电机柱状条
    m1Bar.style.height = `${telem.motorPct[0]}%`;
    m2Bar.style.height = `${telem.motorPct[1]}%`;
    m3Bar.style.height = `${telem.motorPct[2]}%`;
    m4Bar.style.height = `${telem.motorPct[3]}%`;
  },

  onTxFrame: (frame: Uint8Array, txRateHz: number) => {
    hexFrameView.textContent = formatHexFrame(frame);
    txHzVal.textContent = `${txRateHz} Hz`;
  },

  onError: (msg: string) => {
    showAlert(msg);
  },
});

// ==================== 安全解锁联锁与急停按钮逻辑 ====================
function setArmedState(targetArmed: boolean): void {
  if (targetArmed) {
    // 硬性安全红线 S-3：油门高于 5% 严禁解锁！
    if (leftStick.getY() > SAFETY_ARM_MAX_THROTTLE) {
      showAlert('安全联锁拦截：请先把左摇杆油门拉到最低 (< 5%) 再点击解锁！');
      return;
    }
    isKillLatch = false;
    isArmed = true;
    btnArmToggle.textContent = '已解锁 (点击上锁 DISARM)';
    btnArmToggle.classList.add('armed');
  } else {
    isArmed = false;
    btnArmToggle.textContent = '解锁起飞 (ARM) — 需油门 ≤ 5%';
    btnArmToggle.classList.remove('armed');
  }
}

btnArmToggle.addEventListener('click', () => {
  setArmedState(!isArmed);
});

// 红色急停按钮：最高优先级瞬发 KILL 帧 + 油门归零 + 上锁
btnKill.addEventListener('click', () => {
  isKillLatch = true;
  setArmedState(false);
  leftStick.setThrottleZero();
  rightStick.releaseSpringAxes();
  bleController.sendEmergencyKillBurst();
  showAlert('已触发紧急急停 (KILL)！油门已归零并强制上锁。');
});

btnZeroThr.addEventListener('click', () => {
  leftStick.setThrottleZero();
});

btnRateMode.addEventListener('click', () => {
  isHalfRate = !isHalfRate;
  btnRateMode.textContent = isHalfRate ? '舵量: 新手 (±12.5°)' : '舵量: 标准 (±25°)';
  updateStickReadouts();
});

btnBleConnect.addEventListener('click', () => {
  const mode = bleController.getMode();
  if (mode === 'ble-web' || mode === 'ble-native') {
    bleController.disconnect();
  } else {
    bleController.connectBle();
  }
});

btnSimToggle.addEventListener('click', () => {
  if (bleController.getMode() === 'simulator') {
    bleController.disconnect();
  } else {
    bleController.startSimulator();
  }
});

// ==================== 微调按钮 (Trim) ====================
function updateTrimLabels(): void {
  trimRollVal.textContent = `${rollTrimDeg >= 0 ? '+' : ''}${rollTrimDeg.toFixed(1)}°`;
  trimPitchVal.textContent = `${pitchTrimDeg >= 0 ? '+' : ''}${pitchTrimDeg.toFixed(1)}°`;
  updateStickReadouts();
}

document.getElementById('trim-roll-dec')!.addEventListener('click', () => {
  rollTrimDeg = Math.max(-5.0, +(rollTrimDeg - 0.2).toFixed(1));
  updateTrimLabels();
});
document.getElementById('trim-roll-inc')!.addEventListener('click', () => {
  rollTrimDeg = Math.min(5.0, +(rollTrimDeg + 0.2).toFixed(1));
  updateTrimLabels();
});
document.getElementById('trim-roll-rst')!.addEventListener('click', () => {
  rollTrimDeg = 0.0;
  updateTrimLabels();
});

document.getElementById('trim-pitch-dec')!.addEventListener('click', () => {
  pitchTrimDeg = Math.max(-5.0, +(pitchTrimDeg - 0.2).toFixed(1));
  updateTrimLabels();
});
document.getElementById('trim-pitch-inc')!.addEventListener('click', () => {
  pitchTrimDeg = Math.min(5.0, +(pitchTrimDeg + 0.2).toFixed(1));
  updateTrimLabels();
});
document.getElementById('trim-pitch-rst')!.addEventListener('click', () => {
  pitchTrimDeg = 0.0;
  updateTrimLabels();
});

// ==================== 仿真器测试栏按钮 ====================
document.getElementById('sim-vbat-39')!.addEventListener('click', () => bleController.setSimBatteryVoltage(3.95));
document.getElementById('sim-vbat-34')!.addEventListener('click', () => bleController.setSimBatteryVoltage(3.40));
document.getElementById('sim-vbat-32')!.addEventListener('click', () => bleController.setSimBatteryVoltage(3.20));
document.getElementById('sim-crash-tilt')!.addEventListener('click', () => bleController.triggerSimTiltCrash());

// ==================== 页面切后台 / 锁屏自动回中安全保护 ====================
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    rightStick.releaseSpringAxes();
    leftStick.releaseSpringAxes();
  }
});
window.addEventListener('blur', () => {
  rightStick.releaseSpringAxes();
  leftStick.releaseSpringAxes();
});

// ==================== 物理游戏手柄 (Web Gamepad API) 接入 ====================
const applyDeadzone = (v: number, dz = 0.05): number => (Math.abs(v) > dz ? v : 0.0);
let prevKillBtnPressed = false;

new GamepadPoller(
  (pad: Gamepad) => {
    gamepadStatus.textContent = `已连接实体手柄: ${pad.id.slice(0, 22)}`;
    // 标准手柄映射：axes[0]=左X(Yaw), axes[1]=左Y(Throttle), axes[2]=右X(Roll), axes[3]=右Y(Pitch)
    const lx = applyDeadzone(pad.axes[0] ?? 0);
    const ly = applyDeadzone(-(pad.axes[1] ?? 0)); // 上推为正
    const rx = applyDeadzone(pad.axes[2] ?? 0);
    const ry = applyDeadzone(-(pad.axes[3] ?? 0));

    // 右摇杆 (Roll / Pitch)：只要手指没在屏幕上拖拽，每帧同步（含松手归零 0.0！）
    if (!rightStick.isDragging()) {
      rightStick.setNormalized(rx, ry);
    }

    // 左摇杆：横向 Yaw 每帧同步（含松手归零），纵向 Throttle 仅在拨动超过死区时更新
    if (!leftStick.isDragging()) {
      if (ly !== 0.0) {
        leftStick.setNormalized(lx, ly);
      } else {
        leftStick.setXNormalized(lx);
      }
    }

    // B 键 (buttons[1]) 映射为紧急上锁急停（边沿触发，防按住连发）
    const killPressed = Boolean(pad.buttons[1]?.pressed);
    if (killPressed && !prevKillBtnPressed) {
      btnKill.click();
    }
    prevKillBtnPressed = killPressed;

    updateStickReadouts();
  },
  () => {
    gamepadStatus.textContent = '实体手柄已断开（已自动回中）';
    rightStick.releaseSpringAxes();
    leftStick.releaseSpringAxes();
    prevKillBtnPressed = false;
  }
);

// 默认启动离线仿真闭环，方便打开页面即刻体验摇杆与安全状态机
updateStickReadouts();
bleController.startSimulator();
