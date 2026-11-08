/**
 * ble_adapter.ts —— 跨平台低延迟蓝牙传输适配层（Web Bluetooth + Capacitor Native BLE + 离线飞控仿真器）
 *
 * 【核心设计】：
 * 1. 自动识别运行环境：
 *    - 若在 Capacitor 原生壳（Android / iOS App）中运行，通过 Capacitor BLE 插件调用原生蓝牙（支持 requestConnectionPriority HIGH）；
 *    - 若在桌面/安卓 Chrome/Edge 浏览器中运行，调用标准 Web Bluetooth API (`navigator.bluetooth`)；
 *    - 提供“离线飞控仿真模式 (SIM)”：内部复用 `QuadSimulator`，严格复刻 `safety.h` 与 `mixer.h` 物理规律。
 *
 * 2. 50Hz (20ms) 零积压发送引擎：
 *    - 绝不在手指滑动事件中直接发包；
 *    - 固定 20ms 定时器采样最新状态，若上一帧 BLE 异步写入尚未返回（txInFlight = true），则跳过本拍，确保系统蓝牙队列永远零积压、最低延迟。
 */

import {
  type ControlInputState,
  type TelemetryState,
  UAV_BLE_DEVICE_NAME,
  UAV_BLE_SERVICE_UUID,
  UAV_BLE_CHAR_CTRL_UUID,
  UAV_BLE_CHAR_TELEM_UUID,
  PROTOCOL_TELEM_FRAME_SIZE,
  protocolCrc8,
  encodeControlFrame,
  decodeTelemetryFrame,
} from './protocol.ts';
import { QuadSimulator } from './flight_logic.ts';

export type ConnectionMode = 'disconnected' | 'connecting' | 'ble-web' | 'ble-native' | 'simulator';

export interface BleAdapterCallbacks {
  onModeChange: (mode: ConnectionMode, deviceName: string) => void;
  onTelemetry: (telem: TelemetryState) => void;
  onTxFrame: (frame: Uint8Array, txRateHz: number) => void;
  onError: (msg: string) => void;
}

// 兼容 TS 5.0~5.8+ 的二进制写入缓冲类型，规避 TS 5.7+ Uint8Array<ArrayBufferLike> 与 BufferSource 的泛型冲突
type BleWriteBuffer = ArrayBuffer | ArrayBufferView;

interface WebBleCharacteristic {
  writeValueWithoutResponse?: (val: BleWriteBuffer) => Promise<void>;
  writeValue: (val: BleWriteBuffer) => Promise<void>;
  startNotifications: () => Promise<WebBleCharacteristic>;
  addEventListener: (type: string, listener: (ev: Event) => void) => void;
  value?: DataView;
}

interface WebBleGattServer {
  connected: boolean;
  connect: () => Promise<{
    getPrimaryService: (uuid: string) => Promise<{
      getCharacteristic: (uuid: string) => Promise<WebBleCharacteristic>;
    }>;
  }>;
  disconnect: () => void;
}

interface WebBleDevice {
  name?: string;
  gatt?: WebBleGattServer;
  addEventListener: (type: string, listener: () => void) => void;
}

interface CapacitorBleClientLike {
  disconnect: (deviceId: string) => Promise<void>;
  writeWithoutResponse: (
    deviceId: string,
    service: string,
    characteristic: string,
    value: DataView
  ) => Promise<void>;
}

export class UavBleController {
  private mode: ConnectionMode = 'disconnected';
  private deviceName = '未连接';
  private callbacks: BleAdapterCallbacks;

  // Web Bluetooth 句柄
  private webDevice: WebBleDevice | null = null;
  private webCtrlChar: WebBleCharacteristic | null = null;

  // Capacitor Native BLE 句柄
  private capBle: CapacitorBleClientLike | null = null;
  private capDeviceId: string | null = null;

  // 50Hz 定时发送引擎状态
  private txTimerId: ReturnType<typeof setInterval> | null = null;
  private txSeq = 0;
  private txInFlight = false;
  private txCountInWindow = 0;
  private txWindowStartMs = performance.now();
  private measuredTxHz = 0;

  // 离线飞控仿真器实例与遥测序号
  private sim = new QuadSimulator();
  private simSeq = 0;

  private getInputCallback: () => ControlInputState;

  constructor(getInput: () => ControlInputState, callbacks: BleAdapterCallbacks) {
    this.getInputCallback = getInput;
    this.callbacks = callbacks;
  }

  public getMode(): ConnectionMode {
    return this.mode;
  }

  public getDeviceName(): string {
    return this.deviceName;
  }

  public setSimBatteryVoltage(v: number): void {
    this.sim.vbatV = v;
  }

  /**
   * 注入 75° 中度阵风冲击（验证 E-02：60°~110° 窗口保留 45% 油门自稳救机）
   */
  public triggerSimGustTilt(): void {
    if (this.mode === 'simulator') {
      this.sim.rollDeg = 75.0;
    }
  }

  /**
   * 注入 >110° 极端倒扣翻机（验证 E-02：>110° 单拍直切 FAILSAFE 切断电机）
   */
  public triggerSimTiltCrash(): void {
    if (this.mode === 'simulator') {
      this.sim.rollDeg = 120.0;
    }
  }

  /**
   * 连接真实 ESP32-S3 BLE 飞控（自动判断 Capacitor 原生 App 或 Web Bluetooth）
   */
  public async connectBle(): Promise<void> {
    this.disconnect();
    this.setMode('connecting', '正在搜索 UAV-ESP32S3...');

    const cap = (globalThis as unknown as { Capacitor?: { isNativePlatform?: () => boolean } }).Capacitor;
    const isNativeApp = Boolean(cap && typeof cap.isNativePlatform === 'function' && cap.isNativePlatform());

    try {
      if (isNativeApp) {
        await this.connectCapacitorNative();
      } else {
        await this.connectWebBluetooth();
      }
    } catch (err: unknown) {
      this.setMode('disconnected', '未连接');
      const msg = err instanceof Error ? err.message : String(err);
      if (!msg.includes('cancelled') && !msg.includes('User cancelled')) {
        this.callbacks.onError(`蓝牙连接失败: ${msg}`);
      }
    }
  }

  /**
   * 开启离线飞控闭环仿真模式（100% 复现 firmware/src/flight/safety.h 与 mixer.h 行为）
   */
  public startSimulator(): void {
    this.disconnect();
    this.sim.reset();
    this.setMode('simulator', `${UAV_BLE_DEVICE_NAME} (仿真闭环)`);
    this.start50HzLoop();
  }

  public disconnect(): void {
    this.stop50HzLoop();

    // 先清空当前设备引用再调用底层 disconnect()，防止异步 gattserverdisconnected 回调误报异常断链弹窗
    const prevWebDevice = this.webDevice;
    this.webDevice = null;
    this.webCtrlChar = null;
    if (prevWebDevice && prevWebDevice.gatt?.connected) {
      try {
        prevWebDevice.gatt.disconnect();
      } catch {
        // 忽略断开异常
      }
    }

    const prevCapBle = this.capBle;
    const prevCapDeviceId = this.capDeviceId;
    this.capDeviceId = null;
    if (prevCapBle && prevCapDeviceId) {
      void prevCapBle.disconnect(prevCapDeviceId).catch(() => {});
    }

    this.setMode('disconnected', '未连接');
  }

  /**
   * 立即连发 3 帧紧急急停包（KILL Switch 最高优先级）
   */
  public async sendEmergencyKillBurst(): Promise<void> {
    const killInput: ControlInputState = {
      throttle: 0,
      roll: 0,
      pitch: 0,
      yaw: 0,
      armed: false,
      kill: true,
      halfRate: false,
    };

    for (let i = 0; i < 3; i++) {
      const frame = encodeControlFrame(killInput, this.txSeq);
      this.txSeq = (this.txSeq + 1) & 0xff;
      this.callbacks.onTxFrame(frame, this.measuredTxHz);

      const frameView = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);

      if (this.mode === 'ble-web' && this.webCtrlChar) {
        try {
          if (this.webCtrlChar.writeValueWithoutResponse) {
            await this.webCtrlChar.writeValueWithoutResponse(frameView);
          } else {
            await this.webCtrlChar.writeValue(frameView);
          }
        } catch {
          // 忽略单帧发送冲突
        }
      } else if (this.mode === 'ble-native' && this.capBle && this.capDeviceId) {
        try {
          await this.capBle.writeWithoutResponse(
            this.capDeviceId,
            UAV_BLE_SERVICE_UUID,
            UAV_BLE_CHAR_CTRL_UUID,
            frameView
          );
        } catch {
          // 忽略单帧发送冲突
        }
      } else if (this.mode === 'simulator') {
        this.stepSimulator(killInput);
      }
    }
  }

  private async connectWebBluetooth(): Promise<void> {
    const navBle = (
      navigator as unknown as {
        bluetooth?: {
          requestDevice: (opts: unknown) => Promise<WebBleDevice>;
        };
      }
    ).bluetooth;

    if (!navBle) {
      throw new Error(
        '当前浏览器不支持 Web Bluetooth API。请使用电脑/安卓 Chrome 或 Edge，或打包为 Capacitor 原生 App。'
      );
    }

    const device = await navBle.requestDevice({
      filters: [{ services: [UAV_BLE_SERVICE_UUID] }, { namePrefix: 'UAV-' }],
      optionalServices: [UAV_BLE_SERVICE_UUID],
    });

    if (!device.gatt) {
      throw new Error('未找到设备 GATT 服务接口。');
    }

    this.webDevice = device;

    device.addEventListener('gattserverdisconnected', () => {
      if (this.webDevice !== device) return; // 主动断开不触发告警
      this.webDevice = null;
      this.webCtrlChar = null;
      this.stop50HzLoop();
      this.setMode('disconnected', '连接已断开');
      this.callbacks.onError('飞控蓝牙连接已断开！飞控将在 500ms 内自动切入 FAILSAFE 锁死。');
    });

    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(UAV_BLE_SERVICE_UUID);
    const ctrlChar = await service.getCharacteristic(UAV_BLE_CHAR_CTRL_UUID);
    const telemChar = await service.getCharacteristic(UAV_BLE_CHAR_TELEM_UUID);

    await telemChar.startNotifications();
    telemChar.addEventListener('characteristicvaluechanged', (ev: Event) => {
      if (this.webDevice !== device) return;
      const target = ev.target as unknown as WebBleCharacteristic | null;
      if (target && target.value) {
        const parsed = decodeTelemetryFrame(target.value);
        if (parsed) {
          this.callbacks.onTelemetry(parsed);
        }
      }
    });

    this.webCtrlChar = ctrlChar;
    this.setMode('ble-web', device.name || UAV_BLE_DEVICE_NAME);
    this.start50HzLoop();
  }

  private async connectCapacitorNative(): Promise<void> {
    const { BleClient } = await import('@capacitor-community/bluetooth-le');
    await BleClient.initialize();

    const device = await BleClient.requestDevice({
      services: [UAV_BLE_SERVICE_UUID],
      namePrefix: 'UAV-',
    });

    this.capBle = BleClient;
    this.capDeviceId = device.deviceId;

    await BleClient.connect(device.deviceId, () => {
      if (this.capDeviceId !== device.deviceId) return; // 主动断开不触发告警
      this.capDeviceId = null;
      this.stop50HzLoop();
      this.setMode('disconnected', '连接已断开');
      this.callbacks.onError('飞控蓝牙连接已断开！');
    });

    // 原生 Android 端主动请求 HIGH 连接优先级（1 = CONNECTION_PRIORITY_HIGH, 7.5ms~11.25ms 超低延迟间隔）
    try {
      await BleClient.requestConnectionPriority(device.deviceId, 1);
    } catch {
      // iOS 端不支持主动设置 ConnectionPriority，安全忽略
    }

    await BleClient.startNotifications(
      device.deviceId,
      UAV_BLE_SERVICE_UUID,
      UAV_BLE_CHAR_TELEM_UUID,
      (value: DataView) => {
        if (this.capDeviceId !== device.deviceId) return;
        const parsed = decodeTelemetryFrame(value);
        if (parsed) {
          this.callbacks.onTelemetry(parsed);
        }
      }
    );

    this.setMode('ble-native', device.name || UAV_BLE_DEVICE_NAME);
    this.start50HzLoop();
  }

  private start50HzLoop(): void {
    this.stop50HzLoop();
    this.txSeq = 0;
    this.txInFlight = false;
    this.txCountInWindow = 0;
    this.txWindowStartMs = performance.now();

    // 严格 20ms (50Hz) 定时采样与无阻塞发包
    this.txTimerId = globalThis.setInterval(() => {
      void this.tick50Hz();
    }, 20);
  }

  private stop50HzLoop(): void {
    if (this.txTimerId !== null) {
      clearInterval(this.txTimerId);
      this.txTimerId = null;
    }
    this.measuredTxHz = 0;
  }

  private async tick50Hz(): Promise<void> {
    if (this.mode === 'disconnected' || this.mode === 'connecting') return;
    // 若上一帧蓝牙底层 Promise 尚未返回，直接跳过本拍，杜绝缓冲区排队积压
    if (this.txInFlight) return;

    const input = this.getInputCallback();
    const frame = encodeControlFrame(input, this.txSeq);
    this.txSeq = (this.txSeq + 1) & 0xff;

    // 统计实时发包频率 (Hz)
    this.txCountInWindow++;
    const now = performance.now();
    const elapsed = now - this.txWindowStartMs;
    if (elapsed >= 500) {
      this.measuredTxHz = Math.round((this.txCountInWindow * 1000) / elapsed);
      this.txCountInWindow = 0;
      this.txWindowStartMs = now;
    }

    this.callbacks.onTxFrame(frame, this.measuredTxHz);

    if (this.mode === 'simulator') {
      this.stepSimulator(input);
      return;
    }

    const frameView = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);

    this.txInFlight = true;
    try {
      if (this.mode === 'ble-web' && this.webCtrlChar) {
        if (this.webCtrlChar.writeValueWithoutResponse) {
          await this.webCtrlChar.writeValueWithoutResponse(frameView);
        } else {
          await this.webCtrlChar.writeValue(frameView);
        }
      } else if (this.mode === 'ble-native' && this.capBle && this.capDeviceId) {
        await this.capBle.writeWithoutResponse(
          this.capDeviceId,
          UAV_BLE_SERVICE_UUID,
          UAV_BLE_CHAR_CTRL_UUID,
          frameView
        );
      }
    } catch {
      // 单帧偶发繁忙忽略，等待下一拍最新数据
    } finally {
      this.txInFlight = false;
    }
  }

  /**
   * 内部高保真飞控状态机与混控仿真（严格复刻 safety.h 与 mixer.h）
   */
  private stepSimulator(input: ControlInputState): void {
    const simRes = this.sim.step(input);

    // 打包 19 字节模拟遥测帧并复用真实解码通道验证
    const telemBuf = new Uint8Array(PROTOCOL_TELEM_FRAME_SIZE);
    const view = new DataView(telemBuf.buffer);
    telemBuf[0] = 0x55;
    telemBuf[1] = 0xaa;
    telemBuf[2] = this.simSeq & 0xff;
    this.simSeq = (this.simSeq + 1) & 0xff;
    telemBuf[3] = simRes.state;
    view.setUint16(4, Math.round(simRes.vbatV * 1000), true);
    view.setInt16(6, Math.round(simRes.rollDeg * 10), true);
    view.setInt16(8, Math.round(simRes.pitchDeg * 10), true);
    view.setInt16(10, Math.round(simRes.yawDeg * 10), true);
    telemBuf[12] = Math.round(simRes.motors[0] * 100);
    telemBuf[13] = Math.round(simRes.motors[1] * 100);
    telemBuf[14] = Math.round(simRes.motors[2] * 100);
    telemBuf[15] = Math.round(simRes.motors[3] * 100);
    view.setUint16(16, 980, true); // 模拟 Core1 控制环耗时 980us
    telemBuf[18] = protocolCrc8(telemBuf, PROTOCOL_TELEM_FRAME_SIZE - 1);

    const decoded = decodeTelemetryFrame(telemBuf);
    if (decoded) {
      this.callbacks.onTelemetry(decoded);
    }
  }

  private setMode(mode: ConnectionMode, name: string): void {
    this.mode = mode;
    this.deviceName = name;
    this.callbacks.onModeChange(mode, name);
  }
}
