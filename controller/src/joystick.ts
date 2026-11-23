/**
 * joystick.ts —— 双虚拟摇杆（Pointer Events 多指独立追踪）+ Web Gamepad 实体手柄驱动
 *
 * 【触屏操控】：
 * - 左摇杆纵向控制油门 (0.0 ~ 1.0)，松手保持当前油门，底部启动为 0.0。
 * - 右摇杆：
 *   - 纵向 Y：前后俯仰操作 (-1.0 ~ +1.0)，松手自动回中 0.0（上推前飞 = +1.0，在指令合成层映射为 FRD 负俯仰低头角；下拉后退 = -1.0）
 *   - 横向 X：横滚角 Roll  (-1.0 ~ +1.0)，松手自动回中 0.0（右推右倾 = +1.0，左推左倾 = -1.0）
 *
 * 【手机触屏手感】：采用“相对位移”——落指点即原点，
 * 手指落下时油门/姿态指令绝不跳变，只有拖动才改变（见 flight_logic.ts applyStickDelta）。
 */

import { applyStickDelta } from './flight_logic.ts';

export interface DualStickRawState {
  throttle: number; // 0.0 ~ 1.0
  yaw: number;      // -1.0 ~ +1.0
  pitch: number;    // -1.0 ~ +1.0
  roll: number;     // -1.0 ~ +1.0
  gamepadName: string | null;
}

export class VirtualStick {
  private zoneEl: HTMLElement;
  private touchEl: HTMLElement;
  private knobEl: HTMLElement;
  private springY: boolean;

  private activePointerId: number | null = null;
  private normX = 0.0; // -1.0 ~ +1.0
  private normY = 0.0; // 若 springY=false (油门)，内部映射到 +1.0(底=0%油门) ~ -1.0(顶=100%油门)

  // 落指时的起点（相对位移基准）
  private startPx = 0;
  private startPy = 0;
  private startNormY = 0.0;

  private onChange: () => void;

  constructor(
    zoneEl: HTMLElement,
    knobEl: HTMLElement,
    options: { springY: boolean; touchEl: HTMLElement },
    onChange: () => void
  ) {
    this.zoneEl = zoneEl;
    this.touchEl = options.touchEl;
    this.knobEl = knobEl;
    this.springY = options.springY;
    // 默认安全基准：回中轴从 0.0 起步；非回中油门轴从底部 +1.0 (0% 油门) 起步，杜绝误传 100% 满油门
    this.normY = this.springY ? 0.0 : 1.0;
    this.onChange = onChange;

    this.bindEvents();
    this.renderKnob();
  }

  public getX(): number {
    return this.normX;
  }

  /**
   * 获取纵向值：
   * - 对于自动回中摇杆 (Pitch)：返回 -1.0 ~ +1.0（上推为正）
   * - 对于不回中油门摇杆 (Throttle)：将 +1.0(最底) ~ -1.0(最顶) 归一化为 0.0 ~ 1.0
   */
  public getY(): number {
    if (this.springY) {
      return -this.normY; // 屏幕坐标向下为正，翻转为向上推是正
    }
    return Math.max(0.0, Math.min(1.0, (1.0 - this.normY) * 0.5));
  }

  public setThrottleZero(): void {
    if (!this.springY) {
      this.normY = 1.0; // 屏幕最底部对应 throttle = 0.0
      this.startNormY = 1.0;
      this.renderKnob();
      this.onChange();
    }
  }

  public setThrottle(thr01: number): void {
    if (!this.springY) {
      const clamped = Number.isFinite(thr01) ? Math.max(0.0, Math.min(1.0, thr01)) : 0.0;
      this.normY = 1.0 - 2.0 * clamped;
      this.renderKnob();
    }
  }

  public isDragging(): boolean {
    return this.activePointerId !== null;
  }

  public setNormalized(x: number, yUp: number): void {
    const safeX = Number.isFinite(x) ? x : 0.0;
    const safeY = Number.isFinite(yUp) ? yUp : 0.0;
    this.normX = Math.max(-1.0, Math.min(1.0, safeX));
    this.normY = Math.max(-1.0, Math.min(1.0, -safeY));
    this.renderKnob();
  }

  public releaseSpringAxes(): void {
    this.activePointerId = null;
    this.normX = 0.0;
    if (this.springY) this.normY = 0.0;
    this.zoneEl.classList.remove('active');
    this.renderKnob();
    this.onChange();
  }

  /**
   * 紧急安全复位：释放触控追踪并将所有轴（含非回中油门）全部归零至最低安全位置
   */
  public resetAllAxes(): void {
    this.activePointerId = null;
    this.normX = 0.0;
    this.normY = this.springY ? 0.0 : 1.0;
    this.zoneEl.classList.remove('active');
    this.renderKnob();
    this.onChange();
  }

  private bindEvents(): void {
    const el = this.touchEl;

    el.addEventListener('pointerdown', (e: PointerEvent) => {
      if (this.activePointerId !== null) return;
      e.preventDefault();
      this.activePointerId = e.pointerId;
      el.setPointerCapture(e.pointerId);
      this.zoneEl.classList.add('active');
      // 落指即定原点：此刻不改变任何指令值
      this.startPx = e.clientX;
      this.startPy = e.clientY;
      this.startNormY = this.springY ? 0.0 : this.normY;
    });

    el.addEventListener('pointermove', (e: PointerEvent) => {
      if (e.pointerId !== this.activePointerId) return;
      e.preventDefault();
      this.updateFromPointer(e);
    });

    const handleEnd = (e: PointerEvent) => {
      if (e.pointerId !== this.activePointerId) return;
      e.preventDefault();
      this.releaseSpringAxes();
    };

    el.addEventListener('pointerup', handleEnd);
    el.addEventListener('pointercancel', handleEnd);
    el.addEventListener('lostpointercapture', handleEnd);
    // 长按弹出系统菜单会打断触控，直接屏蔽
    el.addEventListener('contextmenu', (e: Event) => e.preventDefault());
  }

  private updateFromPointer(e: PointerEvent): void {
    const rect = this.zoneEl.getBoundingClientRect();
    // 0.35 * 直径 对应摇杆头自身直径(26%)的 ±135% 偏移，实现手机手指与屏幕摇杆头 1:1 像素跟手
    const maxRadius = Math.min(rect.width, rect.height) * 0.35;
    if (maxRadius <= 1e-3) {
      return;
    }

    const next = applyStickDelta(
      0.0,
      this.startNormY,
      this.springY ? e.clientX - this.startPx : 0.0,
      e.clientY - this.startPy,
      maxRadius,
      this.springY // 右杆圆形限幅，左杆只读纵向位移
    );
    this.normX = next.x;
    this.normY = next.y;

    this.renderKnob();
    this.onChange();
  }

  private renderKnob(): void {
    // CSS translate(%) 相对摇杆头自身尺寸(26%)计算，135% × 26% ≈ 35% 父容器半宽
    const maxOffsetKnobPct = 135;
    const offsetX = this.normX * maxOffsetKnobPct;
    const offsetY = this.normY * maxOffsetKnobPct;
    this.knobEl.style.transform = `translate(${offsetX}%, ${offsetY}%)`;
  }
}

/**
 * Web Gamepad API 物理手柄轮询器（支持 Xbox / PlayStation / 标准蓝牙手柄 / USB 航模控）
 * 内置显式断连事件 + 轮询掉线下降沿双重检测，防止无线手柄休眠掉线漏触发 E-04 安全急停。
 */
export class GamepadPoller {
  private rafId: number | null = null;
  private lastTimestampMs: number | null = null;
  private activePadIndex: number | null = null;
  private onUpdate: (gp: Gamepad, dtSec: number) => void;
  private onDisconnect?: () => void;

  private readonly handleConnected = () => {
    this.start();
  };

  private readonly handleDisconnected = (event: GamepadEvent) => {
    if (event.gamepad.index === this.activePadIndex) {
      this.activePadIndex = null;
      this.lastTimestampMs = null;
      if (this.onDisconnect) {
        this.onDisconnect();
      }
    }
  };

  constructor(onUpdate: (gp: Gamepad, dtSec: number) => void, onDisconnect?: () => void) {
    this.onUpdate = onUpdate;
    this.onDisconnect = onDisconnect;
    window.addEventListener('gamepadconnected', this.handleConnected);
    window.addEventListener('gamepaddisconnected', this.handleDisconnected);
    this.start();
  }

  public destroy(): void {
    window.removeEventListener('gamepadconnected', this.handleConnected);
    window.removeEventListener('gamepaddisconnected', this.handleDisconnected);
    if (this.rafId !== null) {
      cancelAnimationFrame(this.rafId);
      this.rafId = null;
    }
    this.lastTimestampMs = null;
  }

  private start(): void {
    if (this.rafId !== null) return;
    const loop = (ts: number) => {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      let activePad: Gamepad | null = null;
      for (const pad of pads) {
        if (pad && pad.connected) {
          if (!activePad) activePad = pad;
          if (pad.index === this.activePadIndex) {
            activePad = pad;
            break;
          }
        }
      }

      if (this.activePadIndex !== null && activePad?.index !== this.activePadIndex) {
        this.activePadIndex = null;
        this.lastTimestampMs = null;
        this.onDisconnect?.();
      }

      if (activePad) {
        this.activePadIndex = activePad.index;
        const dtSec =
          this.lastTimestampMs !== null
            ? Math.max(0.0, Math.min(0.05, (ts - this.lastTimestampMs) * 0.001))
            : 0.016;
        this.lastTimestampMs = ts;
        this.onUpdate(activePad, dtSec);
      } else {
        this.lastTimestampMs = null;
      }

      this.rafId = requestAnimationFrame(loop);
    };
    this.rafId = requestAnimationFrame(loop);
  }
}
