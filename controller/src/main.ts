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
  formatHexFrame,
} from './protocol.ts';
import {
  CTRL_TILT_STANDARD_DEG,
  type GamepadThrottleMode,
  type SafetyEventReason,
  applyAxisDeadzone,
  buildControlInput,
  computeDirectThrottle,
  computeIncrementalThrottle,
  evaluateLifecycleSafety,
  isArmTiltAllowed,
} from './flight_logic.ts';
import { UavBleController, type ConnectionMode } from './ble_adapter.ts';
import { VirtualStick, GamepadPoller } from './joystick.ts';

// ==================== 控制状态变量 ====================
let isArmed = false;
let isKillLatch = false;
let armRequestTimer: ReturnType<typeof setTimeout> | null = null;
let latestTelemetry: TelemetryState | null = null;
let lastTelemetryMs = 0;
const ARM_CONFIRM_TIMEOUT_MS = 1500;
const TELEMETRY_FRESH_MS = 500;
let gamepadYaw = 0;
let heldYaw: { pointerId: number; button: HTMLButtonElement; direction: number } | null = null;
const TOUCH_YAW_INPUT = 0.5;

// 实体手柄左摇杆油门模式（默认增量稳态，适配 Xbox/PS 弹簧回中手柄；航模 USB 控可切直映）
let gamepadThrMode: GamepadThrottleMode = 'incremental';
let directThrEngaged = false;
let directWaitWarned = false;
let prevTelemState: FlightState = FlightState.LOCKED;

// DOM 引用
const connDot = document.getElementById('conn-dot')!;
const connLabel = document.getElementById('conn-label')!;
const connectionDetail = document.getElementById('connection-detail')!;
const throttleFill = document.getElementById('throttle-fill')!;
const flightStatePill = document.getElementById('flight-state-pill')!;
const vbatBox = document.getElementById('vbat-box')!;
const vbatVal = document.getElementById('vbat-val')!;
const txHzVal = document.getElementById('tx-hz-val')!;
const loopUsVal = document.getElementById('loop-us-val')!;

const btnBleConnect = document.getElementById('btn-ble-connect')!;
const btnSimToggle = document.getElementById('btn-sim-toggle')!;
const btnKill = document.getElementById('btn-kill')!;
const btnZeroThr = document.getElementById('btn-zero-thr')!;
const btnThrMode = document.getElementById('btn-thr-mode')!;
const btnArmToggle = document.getElementById('btn-arm-toggle')!;
const armHint = document.getElementById('arm-hint')!;
const btnSettings = document.getElementById('btn-settings')!;
const btnSettingsClose = document.getElementById('btn-settings-close')!;
const settingsDialog = document.getElementById('settings-dialog') as HTMLDialogElement;

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
const motorValues = [1, 2, 3, 4].map((index) => document.getElementById(`m${index}-value`)!);

const hexFrameView = document.getElementById('hex-frame-view')!;
const gamepadRow = document.getElementById('gamepad-row')!;
const gamepadStatus = document.getElementById('gamepad-status')!;
const simToolbar = document.getElementById('sim-toolbar')!;
const alertBanner = document.getElementById('alert-banner')!;

let alertTimeoutId: ReturnType<typeof setTimeout> | null = null;
function showAlert(msg: string): void {
  alertBanner.textContent = msg;
  alertBanner.style.display = 'block';
  if (alertTimeoutId !== null) clearTimeout(alertTimeoutId);
  alertTimeoutId = globalThis.setTimeout(() => {
    alertBanner.style.display = 'none';
  }, 3800);
}

// ==================== 手机体验：触觉反馈 / 屏幕常亮 / 全屏横屏 ====================
function buzz(pattern: number | number[]): void {
  try {
    navigator.vibrate?.(pattern);
  } catch {
    // 不支持振动的设备（如 iOS Safari）静默忽略
  }
}

interface WakeLockSentinelLike {
  addEventListener: (type: 'release', listener: () => void) => void;
}
let wakeLock: WakeLockSentinelLike | null = null;
async function keepScreenOn(): Promise<void> {
  // 锁屏/熄屏会让页面进入后台并触发安全急停，因此操控期间必须常亮
  try {
    const wl = (navigator as unknown as {
      wakeLock?: { request: (type: 'screen') => Promise<WakeLockSentinelLike> };
    }).wakeLock;
    if (wl && !wakeLock) {
      const sentinel = await wl.request('screen');
      wakeLock = sentinel;
      sentinel.addEventListener('release', () => {
        wakeLock = null;
      });
    }
  } catch {
    // 低电量模式 / 不支持时忽略
  }
}
window.addEventListener('pointerdown', () => void keepScreenOn(), { passive: true });
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) void keepScreenOn();
});

async function toggleImmersive(): Promise<void> {
  try {
    if (document.fullscreenElement) {
      await document.exitFullscreen();
      return;
    }
    await document.documentElement.requestFullscreen({ navigationUI: 'hide' });
  } catch {
    // 不支持全屏（如 iPhone Safari）时忽略
  }
  try {
    await (screen.orientation as unknown as { lock?: (o: string) => Promise<void> }).lock?.('landscape');
  } catch {
    // 不支持锁定方向时，依靠竖屏提示层引导手动横屏
  }
}

// ==================== 触屏油门、姿态与偏航 ====================
const leftStick = new VirtualStick(
  document.getElementById('stick-left-zone')!,
  document.getElementById('stick-left-knob')!,
  { springY: false, touchEl: document.getElementById('stick-left-panel')! },
  updateStickReadouts
);

const rightStick = new VirtualStick(
  document.getElementById('stick-right-zone')!,
  document.getElementById('stick-right-knob')!,
  { springY: true, touchEl: document.getElementById('stick-right-touch')! },
  updateStickReadouts
);

function releaseTouchYaw(): void {
  heldYaw?.button.classList.remove('active');
  heldYaw = null;
}

for (const [id, direction] of [['yaw-left', -1], ['yaw-right', 1]] as const) {
  const button = document.getElementById(id) as HTMLButtonElement;
  button.addEventListener('pointerdown', (event) => {
    if (heldYaw || event.button !== 0 || isKillLatch) return;
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    gamepadYaw = 0;
    heldYaw = { pointerId: event.pointerId, button, direction };
    button.classList.add('active');
    updateStickReadouts();
  });
  for (const type of ['pointerup', 'pointercancel', 'lostpointercapture'] as const) {
    button.addEventListener(type, (event) => {
      if (heldYaw?.button !== button || heldYaw.pointerId !== event.pointerId) return;
      releaseTouchYaw();
      updateStickReadouts();
    });
  }
}

// ==================== 组合当前控制输入（含死区、Expo 与 FRD 映射） ====================
function getCurrentControlInput(forDisplay = false): ControlInputState {
  const inhibitFlightInput = !forDisplay && isArmed &&
    (armRequestTimer !== null || latestTelemetry?.state === FlightState.FAILSAFE);
  return buildControlInput(
    {
      throttle: inhibitFlightInput ? 0 : leftStick.getY(),
      yawRight: inhibitFlightInput ? 0 : (heldYaw ? heldYaw.direction * TOUCH_YAW_INPUT : gamepadYaw),
      rollRight: inhibitFlightInput ? 0 : rightStick.getX(),
      pitchForward: inhibitFlightInput ? 0 : rightStick.getY(),
    },
    {
      armed: isArmed,
      kill: isKillLatch,
      halfRate: false,
      rollTrimDeg: 0.0,
      pitchTrimDeg: 0.0,
    }
  );
}

function updateStickReadouts(): void {
  const input = getCurrentControlInput(true);
  valThr.textContent = `${Math.round(input.throttle * 100)}%`;
  throttleFill.style.height = `${input.throttle * 100}%`;
  valYaw.textContent = `${Math.round(input.yaw * 180)}°/s`;
  valPitch.textContent = `${(input.pitch * CTRL_TILT_STANDARD_DEG).toFixed(1)}°`;
  valRoll.textContent = `${(input.roll * CTRL_TILT_STANDARD_DEG).toFixed(1)}°`;
  updateArmHint();
}

function telemetryIsFresh(): boolean {
  return latestTelemetry !== null && performance.now() - lastTelemetryMs <= TELEMETRY_FRESH_MS;
}

function armBlockReason(): string | null {
  const mode = bleController.getMode();
  const telem = latestTelemetry;
  if (mode === 'connecting') return '正在连接飞控';
  if (mode === 'disconnected') return '连接飞控或开启仿真';
  if (!telem || !telemetryIsFresh()) return '等待飞控遥测';
  if (telem.state === FlightState.FAILSAFE) return '飞控故障锁存 · 等待上锁确认';
  if (telem.state !== FlightState.LOCKED) return '等待飞控上锁确认';
  if (leftStick.getY() > SAFETY_ARM_MAX_THROTTLE) return '先将油门降至 5% 以内';
  if (telem.vbatV < SAFETY_VBAT_WARN_V) return '电池低于 3.50 V · 禁止解锁';
  if (!isArmTiltAllowed(telem.rollDeg, telem.pitchDeg)) return '请放平机身 · 倾角过大';
  return null;
}

function updateArmHint(): void {
  const pending = armRequestTimer !== null;
  const confirmed = isArmed && !pending && latestTelemetry?.state !== FlightState.FAILSAFE;
  let hint: string;
  let label = '长按 解锁';
  let warning = false;
  if (isArmed) {
    label = pending ? '长按 取消解锁' : '长按 上锁';
    if (pending) {
      hint = '等待飞控确认 · 请保持机身静止';
    } else if (!telemetryIsFresh()) {
      hint = '飞控遥测超时';
      warning = true;
    } else if (latestTelemetry?.state === FlightState.FAILSAFE) {
      label = '长按 上锁复位';
      hint = '飞控故障锁存 · 电机已停机';
      warning = true;
    } else if (latestTelemetry?.state === FlightState.LOWBAT_LAND) {
      hint = '低压保护已锁存 · 油门已受限';
      warning = true;
    } else {
      hint = '飞控已确认解锁';
    }
  } else {
    const block = armBlockReason();
    warning = block !== null;
    hint = block ?? (isKillLatch ? '急停已锁存 · 长按重新解锁' : '保持机身静止 · 长按解锁');
  }
  if (armHint.textContent !== hint) armHint.textContent = hint;
  if (btnArmToggle.textContent !== label) btnArmToggle.textContent = label;
  armHint.classList.toggle('warn', warning);
  btnArmToggle.classList.toggle('armed', confirmed);
  const pressed = String(confirmed);
  if (btnArmToggle.getAttribute('aria-pressed') !== pressed) {
    btnArmToggle.setAttribute('aria-pressed', pressed);
  }
}

function clearArmRequest(): void {
  if (armRequestTimer !== null) {
    clearTimeout(armRequestTimer);
    armRequestTimer = null;
  }
}

function resetFlightInputs(): void {
  directThrEngaged = false;
  releaseTouchYaw();
  gamepadYaw = 0;
  rightStick.resetAllAxes();
  leftStick.resetAllAxes();
}

// ==================== 紧急安全撤防 / 急停统一入口 ====================
function triggerEmergencyKill(msg = '已触发紧急急停 (KILL)！油门已归零并强制上锁。'): void {
  cancelArmHold();
  if (settingsDialog.open) {
    settingsDialog.close();
  }
  buzz([120, 60, 120]);
  isKillLatch = true;
  setArmedState(false);
  resetFlightInputs();
  void bleController.sendEmergencyKillBurst();
  showAlert(msg);
}

function handleSafetyEvent(reason: SafetyEventReason): void {
  cancelArmHold();
  const wasArmed = isArmed;
  const outcome = evaluateLifecycleSafety(reason, wasArmed);

  if (outcome.latchKill) {
    isKillLatch = true;
  }
  if (outcome.disarm) {
    if (outcome.alertMessage && settingsDialog.open) {
      settingsDialog.close();
    }
    setArmedState(false);
  }
  if (outcome.zeroThrottle && outcome.releaseSpringAxes) {
    resetFlightInputs();
  }
  if (outcome.sendKillBurst) {
    void bleController.sendEmergencyKillBurst();
  }
  if (outcome.alertMessage) {
    showAlert(outcome.alertMessage);
  }
}

// ==================== 初始化 BLE 与仿真控制器 ====================
const bleController = new UavBleController(getCurrentControlInput, {
  onModeChange: (mode: ConnectionMode, deviceName: string) => {
    latestTelemetry = null;
    prevTelemState = FlightState.LOCKED;
    connectionDetail.textContent = deviceName;
    connLabel.textContent = mode === 'simulator' ? '离线仿真' : deviceName;
    connLabel.title = deviceName;
    connDot.className = 'status-dot';
    btnBleConnect.toggleAttribute('disabled', mode === 'connecting');
    btnSimToggle.toggleAttribute('disabled', mode === 'connecting');
    btnSimToggle.classList.toggle('btn-sim-active', mode === 'simulator');
    btnSimToggle.textContent = mode === 'simulator' ? '停止离线仿真' : '开启离线仿真';
    btnSimToggle.setAttribute('aria-pressed', String(mode === 'simulator'));
    if (mode === 'ble-web' || mode === 'ble-native') {
      connDot.classList.add('online');
      btnBleConnect.textContent = '断开 BLE';
      simToolbar.style.display = 'none';
    } else if (mode === 'simulator') {
      connDot.classList.add('sim');
      btnBleConnect.textContent = '连接蓝牙';
      simToolbar.style.display = 'flex';
    } else {
      btnBleConnect.textContent = mode === 'connecting' ? '连接中…' : '连接蓝牙';
      simToolbar.style.display = 'none';
      vbatVal.textContent = '— V';
      vbatBox.className = 'metric-box';
      flightStatePill.textContent = mode === 'connecting' ? '连接中' : '未连接';
      flightStatePill.className = 'state-pill state-locked';
      attRoll.textContent = attPitch.textContent = attYaw.textContent = '—';
      horizonPlate.style.transform = '';
      m1Bar.style.height = m2Bar.style.height = m3Bar.style.height = m4Bar.style.height = '0%';
      motorValues.forEach((element) => { element.textContent = '—'; });
      txHzVal.textContent = '0 Hz';
      loopUsVal.textContent = '— us';
      // 断线立即执行安全上锁与油门归零
      handleSafetyEvent('ble-disconnect');
    }
    updateArmHint();
  },

  onTelemetry: (telem: TelemetryState) => {
    latestTelemetry = telem;
    lastTelemetryMs = performance.now();
    if (armRequestTimer !== null && telem.state !== FlightState.LOCKED) {
      clearArmRequest();
      resetFlightInputs();
    } else if (isArmed && armRequestTimer === null && telem.state === FlightState.LOCKED && prevTelemState !== FlightState.LOCKED) {
      // 飞控已上锁，停止发送解锁位，避免旧请求触发再次解锁。
      setArmedState(false);
      resetFlightInputs();
      showAlert('飞控报告已上锁。重新解锁需再次长按。');
    }
    // 0. 进入故障/迫降态时手机强振提醒（只在状态跳变时触发一次）
    if (telem.state !== prevTelemState) {
      if (telem.state === FlightState.FAILSAFE || telem.state === FlightState.LOWBAT_LAND) {
        buzz([200, 80, 200, 80, 200]);
      } else if (telem.state === FlightState.LOWBAT_WARN) {
        buzz([100, 60, 100]);
      }
      prevTelemState = telem.state;
    }

    // 1. 更新安全状态徽章
    flightStatePill.textContent = FLIGHT_STATE_LABELS[telem.state].replace(/^.*\((.*)\)$/, '$1');
    flightStatePill.title = FLIGHT_STATE_LABELS[telem.state];
    flightStatePill.className = 'state-pill';
    if (telem.state === FlightState.LOCKED) {
      flightStatePill.classList.add('state-locked');
    } else if (telem.state === FlightState.ARMED) {
      flightStatePill.classList.add('state-armed');
    } else if (telem.state === FlightState.LOWBAT_WARN) {
      flightStatePill.classList.add('state-warn');
    } else {
      flightStatePill.classList.add('state-failsafe');
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

    // 4. 更新人工地平仪与欧拉角（FRD：+Roll 右倾地平线逆时针旋，+Pitch 抬头天际线下移露蓝天）
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
    motorValues.forEach((element, index) => {
      element.textContent = `${Math.round(telem.motorPct[index])}%`;
    });
    updateArmHint();
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
    const block = armBlockReason();
    if (block) {
      showAlert(block);
      return;
    }
    isKillLatch = false;
    isArmed = true;
    armRequestTimer = globalThis.setTimeout(() => {
      setArmedState(false);
      resetFlightInputs();
      showAlert('解锁未获确认。请保持机身平放静止，待飞控就绪后重试。');
    }, ARM_CONFIRM_TIMEOUT_MS);
  } else {
    clearArmRequest();
    isArmed = false;
    releaseTouchYaw();
    gamepadYaw = 0;
    updateStickReadouts();
  }
  updateArmHint();
}

// 解锁/上锁必须长按 0.6s（带进度条）：手机上拇指擦碰不会误解锁，更不会在飞行中误上锁断电机；
// 真正的紧急情况用单击 KILL。键盘/辅助功能触发的 click（detail === 0）仍可直接切换。
const ARM_HOLD_MS = 600;
let armHoldTimer: ReturnType<typeof setTimeout> | null = null;

function cancelArmHold(): void {
  if (armHoldTimer !== null) {
    clearTimeout(armHoldTimer);
    armHoldTimer = null;
  }
  btnArmToggle.classList.remove('holding');
}

btnArmToggle.addEventListener('pointerdown', (e: PointerEvent) => {
  e.preventDefault();
  cancelArmHold();
  btnArmToggle.classList.add('holding');
  armHoldTimer = globalThis.setTimeout(() => {
    armHoldTimer = null;
    btnArmToggle.classList.remove('holding');
    setArmedState(!isArmed);
    buzz(isArmed ? [30, 40, 30] : [60]);
  }, ARM_HOLD_MS);
});
for (const evt of ['pointerup', 'pointercancel', 'pointerleave'] as const) {
  btnArmToggle.addEventListener(evt, cancelArmHold);
}
btnArmToggle.addEventListener('click', (e: MouseEvent) => {
  if (e.detail === 0) {
    setArmedState(!isArmed);
  }
});
btnArmToggle.addEventListener('contextmenu', (e: Event) => e.preventDefault());

document.getElementById('btn-fullscreen')!.addEventListener('click', () => {
  void toggleImmersive();
});
// 禁止整页长按弹出系统菜单（会打断触控并可能触发 blur）
document.addEventListener('contextmenu', (e: Event) => e.preventDefault());

// 红色急停按钮：最高优先级瞬发 KILL 帧 + 油门归零 + 上锁
btnKill.addEventListener('click', () => {
  triggerEmergencyKill();
});
document.getElementById('btn-dialog-kill')!.addEventListener('click', () => {
  triggerEmergencyKill();
});

// 状态 / 工具弹窗：关闭按钮、点遮罩背景、Esc 都可顺畅退出，彻底杜绝卡死
btnSettings.addEventListener('click', () => {
  releaseTouchYaw();
  gamepadYaw = 0;
  rightStick.releaseSpringAxes();
  leftStick.releaseSpringAxes();
  updateStickReadouts();
  if (typeof settingsDialog.showModal === 'function') {
    settingsDialog.showModal();
  } else {
    settingsDialog.setAttribute('open', '');
  }
  settingsDialog.querySelector('.dialog-body')!.scrollTop = 0;
});

btnSettingsClose.addEventListener('click', () => {
  settingsDialog.close();
});

settingsDialog.addEventListener('click', (e: MouseEvent) => {
  if (e.target === settingsDialog) {
    settingsDialog.close();
  }
});

settingsDialog.addEventListener('cancel', (e: Event) => {
  e.preventDefault();
  settingsDialog.close();
  if (isArmed) {
    triggerEmergencyKill('键盘 Esc 触发紧急急停 (KILL)！');
  }
});

let zeroThrHintTimer: ReturnType<typeof setTimeout> | null = null;
btnZeroThr.addEventListener('click', () => {
  directThrEngaged = false;
  leftStick.setThrottleZero();
  btnZeroThr.textContent = '已归零 ✓';
  if (zeroThrHintTimer !== null) clearTimeout(zeroThrHintTimer);
  zeroThrHintTimer = globalThis.setTimeout(() => {
    btnZeroThr.textContent = '油门一键归零';
  }, 900);
});

btnThrMode.addEventListener('click', () => {
  gamepadThrMode = gamepadThrMode === 'incremental' ? 'direct' : 'incremental';
  directThrEngaged = false;
  directWaitWarned = false;
  btnThrMode.textContent = gamepadThrMode === 'incremental' ? '油门模式: 增量' : '油门模式: 直映';
});

btnBleConnect.addEventListener('click', () => {
  const mode = bleController.getMode();
  if (mode === 'ble-web' || mode === 'ble-native') {
    if (isArmed) {
      triggerEmergencyKill('断开 BLE 前已自动强制上锁并归零油门。');
    }
    bleController.disconnect();
  } else {
    bleController.connectBle();
  }
});

btnSimToggle.addEventListener('click', () => {
  if (isArmed) {
    triggerEmergencyKill('切换仿真模式前已急停并上锁，油门已归零。');
  }
  if (bleController.getMode() === 'simulator') {
    bleController.disconnect();
  } else {
    bleController.startSimulator();
  }
});

// ==================== 仿真器测试栏按钮 ====================
document.getElementById('sim-vbat-39')!.addEventListener('click', () => bleController.setSimBatteryVoltage(3.95));
document.getElementById('sim-vbat-34')!.addEventListener('click', () => bleController.setSimBatteryVoltage(3.40));
document.getElementById('sim-vbat-32')!.addEventListener('click', () => bleController.setSimBatteryVoltage(3.20));
document.getElementById('sim-gust-tilt')?.addEventListener('click', () => bleController.triggerSimGustTilt());
document.getElementById('sim-crash-tilt')!.addEventListener('click', () => bleController.triggerSimTiltCrash());
document.getElementById('sim-control-loss')!.addEventListener('click', () => bleController.interruptSimControl());

// ==================== E-04 页面切后台 / 失焦 / 离开页面 自动上锁 + 油门归零 + 急停 ====================
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    handleSafetyEvent('visibility-hidden');
  }
});
window.addEventListener('blur', () => {
  handleSafetyEvent('window-blur');
});
window.addEventListener('pagehide', () => {
  handleSafetyEvent('pagehide');
});

// 笔记本键盘急停兜底：Esc 随时急停（弹窗未解锁时先关弹窗）；已解锁状态下空格键优先触发急停
window.addEventListener('keydown', (e: KeyboardEvent) => {
  if (e.code !== 'Escape' && e.code !== 'Space') return;
  e.preventDefault(); // 防止焦点落在解锁按钮时，空格产生原生 click 误解锁。
  if (e.repeat) return;
  if (e.code === 'Escape') {
    if (settingsDialog.open && !isArmed) {
      settingsDialog.close();
      return;
    }
    e.preventDefault();
    triggerEmergencyKill('键盘快捷键触发紧急急停 (KILL)！');
  } else if (e.code === 'Space' && isArmed) {
    e.preventDefault();
    triggerEmergencyKill('键盘空格触发紧急急停 (KILL)！');
  }
});

// ==================== 物理游戏手柄 (Web Gamepad API) 接入 ====================
let prevKillBtnPressed = false;

new GamepadPoller(
  (pad: Gamepad, dtSec: number) => {
    gamepadRow.hidden = false;
    gamepadStatus.textContent = `手柄: ${pad.id.slice(0, 16)}`;

    // 标准手柄映射：axes[0]=左X(Yaw), axes[1]=左Y(Throttle), axes[2]=右X(Roll), axes[3]=右Y(PitchForward)
    const lx = applyAxisDeadzone(pad.axes[0] ?? 0, 0.05);
    const rawLeftYUp = -(pad.axes[1] ?? 0); // 上推为正
    const rx = applyAxisDeadzone(pad.axes[2] ?? 0, 0.05);
    const ryForward = applyAxisDeadzone(-(pad.axes[3] ?? 0), 0.05);

    // 右摇杆 (Roll / Pitch)：只要手指没在屏幕上拖拽，每帧显式同步（含死区显式写入 0.0！）
    if (!rightStick.isDragging()) {
      rightStick.setNormalized(rx, ryForward);
    }

    // 实体手柄保留 Mode 2。触屏操控优先，实体偏航回中时显式写入 0.0。
    gamepadYaw = heldYaw || leftStick.isDragging() || rightStick.isDragging() || isKillLatch ? 0 : lx;
    if (!leftStick.isDragging()) {
      if (gamepadThrMode === 'incremental') {
        const nextThr = computeIncrementalThrottle(leftStick.getY(), rawLeftYUp, dtSec, 0.08);
        leftStick.setThrottle(nextThr);
      } else {
        const res = computeDirectThrottle(rawLeftYUp, leftStick.getY(), directThrEngaged);
        directThrEngaged = res.engaged;
        if (res.engaged) {
          directWaitWarned = false;
          leftStick.setThrottle(res.throttle);
        } else if (!directWaitWarned && isArmed) {
          directWaitWarned = true;
          showAlert('直映模式防暴冲保护：请先把实体左摇杆拉回最低 (≤5%) 后再接管！');
        }
      }
    }

    // B 键 (buttons[1]) 映射为紧急上锁急停（边沿触发，防按住连发）
    const killPressed = Boolean(pad.buttons[1]?.pressed);
    if (killPressed && !prevKillBtnPressed) {
      triggerEmergencyKill('实体手柄急停键 (B) 触发紧急停机！油门已归零并上锁。');
    }
    prevKillBtnPressed = killPressed;

    updateStickReadouts();
  },
  () => {
    prevKillBtnPressed = false;
    directThrEngaged = false;
    directWaitWarned = false;
    gamepadRow.hidden = true;
    handleSafetyEvent('gamepad-disconnect');
  }
);

// 默认启动离线仿真闭环，方便打开页面即刻体验摇杆与安全状态机
updateStickReadouts();
bleController.startSimulator();
globalThis.setInterval(updateArmHint, 250);
