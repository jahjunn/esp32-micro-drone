/**
 * shims.d.ts —— 外部可选原生插件的离线类型声明兜底
 * 作用：在尚未执行 `npm install` 安装 node_modules 时，保证 VS Code / TypeScript 语言服务零报错，
 *       且严格约束 `@capacitor-community/bluetooth-le` BleClient 的真实函数签名。
 */
declare module '@capacitor-community/bluetooth-le' {
  export interface BleDevice {
    deviceId: string;
    name?: string;
  }

  export interface RequestBleDeviceOptions {
    services?: string[];
    optionalServices?: string[];
    name?: string;
    namePrefix?: string;
  }

  export enum ConnectionPriority {
    CONNECTION_PRIORITY_BALANCED = 0,
    CONNECTION_PRIORITY_HIGH = 1,
    CONNECTION_PRIORITY_LOW_POWER = 2,
  }

  export const BleClient: {
    initialize(options?: { androidNeverForLocation?: boolean }): Promise<void>;
    requestDevice(options?: RequestBleDeviceOptions): Promise<BleDevice>;
    connect(deviceId: string, onDisconnect?: (deviceId: string) => void): Promise<void>;
    disconnect(deviceId: string): Promise<void>;
    requestConnectionPriority(deviceId: string, connectionPriority: ConnectionPriority | number): Promise<void>;
    startNotifications(
      deviceId: string,
      service: string,
      characteristic: string,
      callback: (value: DataView) => void
    ): Promise<void>;
    stopNotifications(deviceId: string, service: string, characteristic: string): Promise<void>;
    writeWithoutResponse(
      deviceId: string,
      service: string,
      characteristic: string,
      value: DataView
    ): Promise<void>;
  };
}
