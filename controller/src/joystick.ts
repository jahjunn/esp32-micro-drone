/**
 * joystick.ts —— 双虚拟摇杆（Pointer Events 多指独立追踪）+ Web Gamepad 实体手柄驱动
 *
 * 【美国手 Mode 2 标准定义】：
 * - 左摇杆 (Left Stick)：
 *   - 纵向 Y：油门 Throttle (0.0 ~ 1.0)，**松手不回中**（保持当前推力高度，底部启动为 0.0）
 *   - 横向 X：偏航角速度 Yaw (-1.0 ~ +1.0)，松手自动回中 0.0
 * - 右摇杆 (Right Stick)：
 *   - 纵向 Y：俯仰角 Pitch (-1.0 ~ +1.0)，松手自动回中 0.0（上推前倾为正，下拉后仰为负）
 *   - 横向 X：横滚角 Roll  (-1.0 ~ +1.0)，松手自动回中 0.0（右推右倾为正，左推左倾为负）
 */

export interface DualStickRawState {
  throttle: number; // 0.0 ~ 1.0
  yaw: number;      // -1.0 ~ +1.0
  pitch: number;    // -1.0 ~ +1.0
  roll: number;     // -1.0 ~ +1.0
  gamepadName: string | null;
}

export class VirtualStick {
  private zoneEl: HTMLElement;
  private knobEl: HTMLElement;
  private springX: boolean;
  private springY: boolean;

  private activePointerId: number | null = null;
  private normX = 0.0; // -1.0 ~ +1.0
  private normY = 0.0; // 若 springY=false (油门)，内部映射到 -1.0(底) ~ +1.0(顶)

  private onChange: () => void;

  constructor(
    zoneEl: HTMLElement,
    knobEl: HTMLElement,
    options: { springX: boolean; springY: boolean; initialY?: number },
    onChange: () => void
  ) {
    this.zoneEl = zoneEl;
    this.knobEl = knobEl;
    this.springX = options.springX;
    this.springY = options.springY;
    this.normY = options.initialY ?? (this.springY ? 0.0 : -1.0);
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
   * - 对于不回中油门摇杆 (Throttle)：将 -1.0(最底) ~ +1.0(最顶) 归一化为 0.0 ~ 1.0
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
      this.renderKnob();
      this.onChange();
    }
  }

  public setNormalized(x: number, yUp: number): void {
    this.normX = Math.max(-1.0, Math.min(1.0, x));
    this.normY = Math.max(-1.0, Math.min(1.0, -yUp));
    this.renderKnob();
  }

  public releaseSpringAxes(): void {
    this.activePointerId = null;
    if (this.springX) this.normX = 0.0;
    if (this.springY) this.normY = 0.0;
    this.zoneEl.classList.remove('active');
    this.renderKnob();
    this.onChange();
  }

  private bindEvents(): void {
    this.zoneEl.addEventListener('pointerdown', (e: PointerEvent) => {
      if (this.activePointerId !== null) return;
      e.preventDefault();
      this.activePointerId = e.pointerId;
      this.zoneEl.setPointerCapture(e.pointerId);
      this.zoneEl.classList.add('active');
      this.updateFromPointer(e);
    });

    this.zoneEl.addEventListener('pointermove', (e: PointerEvent) => {
      if (e.pointerId !== this.activePointerId) return;
      e.preventDefault();
      this.updateFromPointer(e);
    });

    const handleEnd = (e: PointerEvent) => {
      if (e.pointerId !== this.activePointerId) return;
      e.preventDefault();
      this.releaseSpringAxes();
    };

    this.zoneEl.addEventListener('pointerup', handleEnd);
    this.zoneEl.addEventListener('pointercancel', handleEnd);
  }

  private updateFromPointer(e: PointerEvent): void {
    const rect = this.zoneEl.getBoundingClientRect();
    const centerX = rect.left + rect.width * 0.5;
    const centerY = rect.top + rect.height * 0.5;
    const maxRadius = Math.min(rect.width, rect.height) * 0.38;

    const dx = e.clientX - centerX;
    const dy = e.clientY - centerY;

    if (this.springY) {
      // 圆形矢量限幅（右摇杆 Pitch + Roll）
      const dist = Math.hypot(dx, dy);
      const scale = dist > maxRadius ? maxRadius / dist : 1.0;
      this.normX = ((dx * scale) / maxRadius);
      this.normY = ((dy * scale) / maxRadius);
    } else {
      // 左摇杆（油门纵向独立方形行程 + 偏航横向行程，避免打偏航时油门被圆形边界拉低！）
      this.normX = Math.max(-1.0, Math.min(1.0, dx / maxRadius));
      this.normY = Math.max(-1.0, Math.min(1.0, dy / maxRadius));
    }

    this.renderKnob();
    this.onChange();
  }

  private renderKnob(): void {
    const maxOffsetPct = 36; // 摇杆头移动半径占容器百分比
    const offsetX = this.normX * maxOffsetPct;
    const offsetY = this.normY * maxOffsetPct;
    this.knobEl.style.transform = `translate(${offsetX}%, ${offsetY}%)`;
  }
}

/**
 * Web Gamepad API 物理手柄轮询器（支持 Xbox / PlayStation / 标准蓝牙手柄）
 */
export class GamepadPoller {
  private rafId: number | null = null;
  private onUpdate: (gp: Gamepad) => void;

  constructor(onUpdate: (gp: Gamepad) => void) {
    this.onUpdate = onUpdate;
    window.addEventListener('gamepadconnected', () => this.start());
    window.addEventListener('gamepaddisconnected', () => {});
    this.start();
  }

  private start(): void {
    if (this.rafId !== null) return;
    const loop = () => {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      for (const pad of pads) {
        if (pad && pad.connected) {
          this.onUpdate(pad);
          break;
        }
      }
      this.rafId = requestAnimationFrame(loop);
    };
    this.rafId = requestAnimationFrame(loop);
  }
}
