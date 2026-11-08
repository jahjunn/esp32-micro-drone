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
} from './flight_logic.ts';
import { UavBleController, type ConnectionMode } from './ble_adapter.ts';
import { VirtualStick, GamepadPoller } from './joystick.ts';

// ==================== 控制状态变量 ====================
let isArmed = false;
let isKillLatch = false;

// 实体手柄左摇杆油门模式（默认增量稳态，适配 Xbox/PS 弹簧回中手柄；航模 USB 控可切直映）
let gamepadThrMode: GamepadThrottleMode = 'incremental';
let directThrEngaged = false;
let directWaitWarned = false;
let prevTelemState: FlightState = FlightState.LOCKED;

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
const btnThrMode = document.getElementById('btn-thr-mode')!;
const btnArmToggle = document.getElementById('btn-arm-toggle')!;
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

// ==================== 初始化左右双虚拟摇杆 (Mode 2) ====================
// 整块半屏面板作为触控区（拇指落哪都行），相对位移式：手指落下不会让油门/姿态跳变
const leftStick = new VirtualStick(
  document.getElementById('stick-left-zone')!,
  document.getElementById('stick-left-knob')!,
  {
    springX: true,
    springY: false,
    initialY: 1.0, // initialY = 1.0 对应底部 0% 油门
    touchEl: document.getElementById('stick-left-panel')!,
  },
  updateStickReadouts
);

const rightStick = new VirtualStick(
  document.getElementById('stick-right-zone')!,
  document.getElementById('stick-right-knob')!,
  { springX: true, springY: true, initialY: 0.0, touchEl: document.getElementById('stick-right-panel')! },
  updateStickReadouts
);

// ==================== 组合当前控制输入（含死区、Expo 与 Mode-2->FRD 映射） ====================
function getCurrentControlInput(): ControlInputState {
  return buildControlInput(
    {
      throttle: leftStick.getY(),
      yawRight: leftStick.getX(),
      rollRight: rightStick.getX(),
      pitchForward: rightStick.getY(),
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
  const input = getCurrentControlInput();
  valThr.textContent = `${Math.round(input.throttle * 100)}%`;
  valYaw.textContent = `${Math.round(input.yaw * 180)}°/s`;
  valPitch.textContent = `${(input.pitch * CTRL_TILT_STANDARD_DEG).toFixed(1)}°`;
  valRoll.textContent = `${(input.roll * CTRL_TILT_STANDARD_DEG).toFixed(1)}°`;
}

// ==================== 紧急安全撤防 / 急停统一入口 ====================
function triggerEmergencyKill(msg = '已触发紧急急停 (KILL)！油门已归零并强制上锁。'): void {
  if (settingsDialog.open) {
    settingsDialog.close();
  }
  buzz([120, 60, 120]);
  isKillLatch = true;
  setArmedState(false);
  directThrEngaged = false;
  rightStick.resetAllAxes();
  leftStick.resetAllAxes();
  void bleController.sendEmergencyKillBurst();
  showAlert(msg);
}

function handleSafetyEvent(reason: SafetyEventReason): void {
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
    directThrEngaged = false;
    rightStick.resetAllAxes();
    leftStick.resetAllAxes();
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
      // 断线立即执行安全上锁与油门归零
      handleSafetyEvent('ble-disconnect');
    }
  },

  onTelemetry: (telem: TelemetryState) => {
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
        btnArmToggle.textContent = '故障！长按上锁复位';
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
    const mode = bleController.getMode();
    if (mode === 'disconnected' || mode === 'connecting') {
      showAlert('当前未连接飞控或仿真器，请先连接 BLE 或开启离线仿真！');
      return;
    }
    // 硬性安全红线 S-3：油门高于 5% 严禁解锁！
    if (leftStick.getY() > SAFETY_ARM_MAX_THROTTLE) {
      showAlert('安全联锁拦截：请先把左摇杆油门拉到最低 (< 5%) 再点击解锁！');
      return;
    }
    isKillLatch = false;
    isArmed = true;
    btnArmToggle.textContent = '长按 上锁';
    btnArmToggle.classList.add('armed');
  } else {
    isArmed = false;
    btnArmToggle.textContent = '长按 解锁';
    btnArmToggle.classList.remove('armed');
  }
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

// 状态 / 工具弹窗：关闭按钮、点遮罩背景、Esc 都可顺畅退出，彻底杜绝卡死
btnSettings.addEventListener('click', () => {
  if (typeof settingsDialog.showModal === 'function') {
    settingsDialog.showModal();
  } else {
    settingsDialog.setAttribute('open', '');
  }
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

    // 左摇杆：Yaw 显式写入 0.0；油门按选定模式安全更新
    if (!leftStick.isDragging()) {
      leftStick.setXNormalized(lx);

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
