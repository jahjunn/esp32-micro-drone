# 轻量无人机飞控 —— 架构设计文档（ARCHITECTURE）

> 定位：**开源学习参考型**飞控。代码以"清晰、模块化、好读"为第一原则，
> 不求功能最全，但求每一行都看得懂，方便他人照着学。
>
> 本文档定**架构、模块划分、数据流、算法选型与协议契约**，与 `firmware/` 及 `controller/` 源码保持严格一致。

---

## 1. 设计基线（已与用户确认）

| 项目 | 选型 | 备注 |
|------|------|------|
| 主控 | **ESP32-S3-N16R8 开发板**（双核 240MHz，16MB Flash/8MB PSRAM，USB-C，向下焊） | 内置 WiFi/BLE；Core0 通信、Core1 控制；25.5×52.5mm |
| 运行时 | **FreeRTOS 双核** | 通信与控制物理隔离，跨核 `portMUX` 零积压传递 |
| 开发框架 | **Arduino / ESP-IDF（PlatformIO）** | Arduino 上手快，必要时可下沉到 ESP-IDF |
| 主传感器 | **MPU6050**（I²C） | 3 轴加速度 + 3 轴陀螺仪 |
| 磁力计 | **QMC5883P**（I²C 0x2C） | 航向锁定，抑制 yaw 漂移；与 L 版寄存器不兼容 |
| 执行机构 | **4× 8520 空心杯 + 4× AO3400 + SS34** | 1S 直驱，低边 MOS |
| 电机 PWM | **LEDC 20kHz / 10bit** | 有刷高频 PWM（非舵机 50Hz） |
| 电池 | **1S 3.7V 600mAh ≥25C** | 悬停约 4–5A，续航 4–6 min |
| 电源 | **1S → MT3608 升 5V → ESP32**；传感器用板载 3.3V；**VBAT 分压进 ADC** | 1S 直接进 AMS1117 压差不够 |
| 控制链路 | **ESP32-S3 BLE GATT（7.5–15ms 连接间隔）+ 跨平台手柄/地面站（Web Bluetooth / Gamepad / Capacitor）** | 13B 下行控制帧 + 19B 上行遥测帧（CRC-8），10–20m 低延迟 |
| 机架 | **嘉立创异形 PCB（板即机架）** | 100mm 对角轴距 |
| 算法 | **二阶巴特沃斯 LPF + Mahony 四元数 + 串级 PID + 保均值抗饱和 Quad-X** | 与成熟开源方案对齐 |
| 控制频率 | **内环 400Hz / 外环 100Hz / BLE 下行 50Hz / 遥测 10–20Hz** | 见 §6 |
| 安全 | 五态状态机：失联 500ms / 倾角 >60° Sticky 锁死、3.5V/3.3V 分级低压、解锁四重联锁、电机总闸门、物理急停 | 见 §5.6 |
| 构建/IDE | VSCode + PlatformIO（固件）+ Node.js / Vite / Capacitor（地面站） | 固件与地面站双端单测统一管理 |

**技术栈一句话**：ESP32-S3 + FreeRTOS 双核 + Arduino/ESP-IDF，用 PlatformIO 构建，
算法与通信协议用"桌面侧单元测试"先验证 —— 定位**开源学习参考型**项目。

### 1.1 为什么从 STM32 无刷改为 ESP32 有刷 + BLE 直连

| 原因 | 说明 |
|------|------|
| 成本 | 无刷方案 ≈¥193；有刷方案 **≈¥70–78** |
| 复杂度 | 免电调校准、免 BLHeli、免信号地处理；MOS 直驱最直观 |
| 控制链路 | ESP32-S3 自带 BLE 5.0（+9dBm），**省掉外置接收机**，且比 WiFi 射频峰值电流更低、无 TCP 队头阻塞 |
| 算力 | 双核 240MHz，跑 400Hz 控制 + BLE 绰绰有余 |
| 代价 | 空心杯寿命短、效率低、抗风差；但本项目目标只是"能大致飞" |

### 1.2 参考项目（选型依据）

| 项目 | 参考点 |
|------|--------|
| Micro ESP32 Drone（45 元方案） | ESP32 双核分工、网页/移动端触屏遥控、Mahony + LPF、倾角保护 |
| Crazyflie | 空心杯 + PCB 机架 + 小轴距 + BLE 紧凑二进制通信的标杆 |
| Quicksilver / EmuFlight | 串级 PID + 保均值抗饱和 Quad-X 混控的通用结构 |

---

## 2. 系统架构总览

```
        手机 / 电脑浏览器 / Capacitor App / 实体手柄
              │  BLE GATT (50Hz 13B Write-No-Resp / 20Hz 19B Notify)
              ▼
┌───────────────────── ESP32-S3-N16R8 ─────────────────────┐
│  Core0（BLE 通信 / 遥测）         Core1（实时控制 400Hz）  │
│  ┌────────────────┐   portMUX   ┌─────────────────────┐  │
│  │ BLE GATT 服务端│  零积压锁   │ 读 MPU6050          │  │
│  │ CRC8 校验解析  │ ──────────▶ │ 二阶 LPF (30–50Hz)  │  │
│  │ 19B 遥测 Notify│             │ Mahony 四元数解算   │  │
│  ├────────────────┤             │ 五态安全机 safety_t │  │
│  │ 连接/断链监控  │             │ 串级双环 PID        │  │
│  │ 状态灯 (2Hz)   │             │ 保均值 Quad-X 混控  │  │
│  └────────────────┘             │ 电机总闸门 → 4×PWM  │  │
│         ▲                       └──────────┬──────────┘  │
│         │ I²C                              │ 20kHz LEDC  │
│   MPU6050 / QMC5883P (0x2C)      4× AO3400 + SS34 → 8520 │
│         ▲                                                │
│   1S ── MT3608(5V) ── 板载LDO(3.3V) ── VBAT分压(GPIO5)   │
└──────────────────────────────────────────────────────────┘
```

---

## 3. 模块划分（目录结构）

```
轻量无人机/
├── firmware/
│   ├── platformio.ini              # ESP32-S3-N16R8 + Arduino + 16MB Flash / 8MB OPI PSRAM
│   ├── src/
│   │   ├── main.cpp                # 入口：初始化 + 建任务 + 启动（待合拢）
│   │   ├── config.h                # 引脚、频率、PID 初值、阈值（待建）
│   │   ├── drivers/                # 只负责"读数据/发信号"
│   │   │   ├── mpu6050.*           # 原始 accel/gyro（待建）
│   │   │   ├── qmc5883p.h/.cpp     # QMC5883P 磁力计驱动（I²C 0x2C，已完成）
│   │   │   ├── motor_out.*         # LEDC 4 路 20kHz（待建）
│   │   │   ├── vbat.*              # ADC 电池电压（待建）
│   │   │   └── status_led.*        # 状态灯（待建）
│   │   ├── flight/                 # 只负责"算控制量"（纯头文件内联，可直接在 PC 单测）
│   │   │   ├── types.h             # 核心物理量结构体定义（已完成）
│   │   │   ├── biquad_lpf.h        # 二阶巴特沃斯低通滤波（已完成）
│   │   │   ├── mahony.h            # Mahony 四元数姿态解算（已完成）
│   │   │   ├── pid.h               # 串级双环 PID 与抗积分饱和（已完成）
│   │   │   ├── mixer.h             # 保均值抗饱和 Quad-X 混控（已完成）
│   │   │   └── safety.h            # 五态安全状态机与电机总闸门（已完成）
│   │   └── comm/
│   │       ├── protocol.h          # 13B 控制帧 + 19B 遥测帧 + CRC-8 校验（已完成）
│   │       └── ble_server.h/.cpp   # ESP32-S3 BLE GATT 低延迟服务端与跨核锁（已完成）
│   └── test/                       # 桌面侧 C 语言单元测试（native，无硬件，全部通过）
│       ├── test_biquad_lpf.c
│       ├── test_mahony.c
│       ├── test_pid.c
│       ├── test_mixer.c
│       ├── test_safety.c
│       └── test_protocol.c
└── controller/                     # 跨平台低延迟蓝牙手柄与地面站（Web Bluetooth + Gamepad + Capacitor）
    ├── index.html                  # 响应式双摇杆与遥测 HUD 面板
    ├── dev_server.mjs              # 零依赖 Node.js 极速开发服务器
    ├── src/
    │   ├── protocol.ts             # 与 protocol.h 严格字节对齐的编码/解码与摇杆死区/Expo 数学库
    │   ├── ble_adapter.ts          # Web Bluetooth 50Hz 无积压发包器 + 内置离线飞控仿真器 (Simulator)
    │   ├── joystick.ts             # Mode 2 多点触控虚拟摇杆 + 实体手柄 Gamepad 轮询
    │   └── main.ts                 # UI 绑定、Trim 微调、五重安全联锁与遥测渲染
    └── test/
        └── verify_protocol.mjs     # 前端协议编码/CRC-8/摇杆曲线自动化单测
```

**分层原则**：`drivers` 只管硬件读写，`flight` 只管纯数学控制律，`comm` 负责二进制协议与 BLE 跨核同步；
换传感器或换通信物理层互不影响。

---

## 4. 数据流（核心结构体，对齐 `types.h` / `safety.h` / `protocol.h`）

```c
// 1. 三维向量（传感器三轴加速度 m/s^2 或角速度 rad/s）
typedef struct {
    float x, y, z;
} Vector3_t;

// 2. 姿态欧拉角（Mahony 解算输出，弧度制 rad）
typedef struct {
    float roll, pitch, yaw;
} EulerAngle_t;

// 3. 飞控内部遥控目标（由 BLE 13B 控制帧解包转换而来）
typedef struct {
    float throttle;         // 油门归一化 0.0 ~ 1.0
    float roll_target;      // 目标横滚角 rad（默认满杆 ±25°，小舵量模式 ±12.5°）
    float pitch_target;     // 目标俯仰角 rad
    float yaw_rate_target;  // 目标偏航角速度 rad/s（满杆 ±180°/s）
    bool  armed;            // 解锁标志：true=请求解锁，false=上锁
} ControlCommand_t;

// 4. 四路电机混控输出（归一化占空比 0.0 ~ 1.0）
typedef struct {
    float m1; // 右前 (CCW)
    float m2; // 右后 (CW)
    float m3; // 左后 (CCW)
    float m4; // 左前 (CW)
} MotorOutput_t;

// 5. 五态安全状态枚举（safety.h）
typedef enum {
    FLIGHT_STATE_LOCKED = 0,   // 上锁待机（上电默认，电机焊死为 0）
    FLIGHT_STATE_ARMED,        // 正常解锁飞行
    FLIGHT_STATE_LOWBAT_WARN,  // 一级低压告警（<3.5V，允许飞回）
    FLIGHT_STATE_LOWBAT_LAND,  // 二级严重低压（<3.3V，强制回平并限油门 ≤25% 缓降）
    FLIGHT_STATE_FAILSAFE      // 故障急停锁死（失联 >500ms 或倾角 >60°，需拨回上锁复位）
} FlightState_t;
```

### 4.1 二进制空中通信协议（`protocol.h`，小端序 1 字节对齐）

- **下行控制帧 `BleControlPacket_t`（手柄 → 飞控，固定 13 字节，50Hz Write Without Response）**：
  - `[0..1]` `header`: `0xAA, 0x55`
  - `[2]` `seq`: `0~255` 循环包序号
  - `[3]` `flags`: `bit0=ARMED`（解锁），`bit1=KILL`（紧急拍停最高优先级），`bit2=HALF_RATE`（新手半舵量）
  - `[4..5]` `throttle`: `uint16_t` `0 ~ 1000`
  - `[6..11]` `roll / pitch / yaw`: `int16_t` `-1000 ~ +1000`（偶数地址自然 2 字节对齐）
  - `[12]` `crc8`: 前 12 字节 `CRC-8 (poly=0x07)` 校验（帧头、长度、CRC 或量程任一越界直接拒收丢弃）
- **上行遥测帧 `BleTelemetryPacket_t`（飞控 → 手柄，固定 19 字节，10–20Hz Notify）**：
  - `[0..1]` `header`: `0x55, 0xAA`
  - `[2]` `seq`、`[3]` `state` (`FlightState_t`)、`[4..5]` `vbat_mv` (毫伏)
  - `[6..11]` `roll_deg10 / pitch_deg10 / yaw_deg10` (`0.1°` 精度 `int16_t`)
  - `[12..15]` `motor_pct[4]` (`0~100%`)、`[16..17]` `loop_us` (控制环耗时微秒)、`[18]` `crc8`

---

## 5. 算法选型（重点）

### 5.1 数字滤波：二阶巴特沃斯低通（LPF，`biquad_lpf.h`）

空心杯 4 万转以上，机械振动极强，**必须先滤波再进姿态解算**。

- 类型：二阶巴特沃斯（Butterworth），通带平坦、无相位失真尖峰；
- 截止频率：**30–50Hz**（先 40Hz，按实测调）；
- 作用对象：陀螺仪三轴（主要）、加速度计三轴（可选）；
- 实现：双二阶（Direct Form I / II biquad）差分方程，支持三轴独立状态。

### 5.2 姿态解算：Mahony 互补滤波（四元数，`mahony.h`）

- 输入：陀螺角速度 + 加速度计重力向量（含加速度模长有效性窗保护 `0.5g ~ 1.5g`）；
- 输出：四元数 → 欧拉角 `roll / pitch / yaw`；
- 参数：`Kp`（加速度计修正增益）、`Ki`（陀螺零偏积分增益，带积分限幅防发散）；
- 相对欧拉角互补滤波的优势：**无万向锁**、大角度机动更稳。

### 5.3 控制器：串级 PID（`pid.h`）

```
角度设定 ─▶[外环 100Hz]─▶ 目标角速度 ─▶[内环 400Hz]─▶ 力矩 ─▶ 混控
              ▲                              ▲
          当前角度                       陀螺角速度
```

- 三轴各一套：roll / pitch / yaw；
- 内置积分限幅（Anti-windup）与总输出限幅。

### 5.4 混控：保均值抗饱和 Quad-X（`mixer.h`）

电机编号与转向（**必须与实物一致**）：

| 电机 | 位置 | 转向 |
|------|------|------|
| M1 | 右前 | CCW |
| M2 | 右后 | CW |
| M3 | 左后 | CCW |
| M4 | 左前 | CW |

```
M1 = T - P + R + Y
M2 = T + P + R - Y
M3 = T + P - R + Y
M4 = T - P - R - Y
```

- **保均值平移抗饱和（Shift-and-Clamp）**：当高油门叠加大幅度姿态力矩导致某路电机 `> 1.0`（或低油门时 `< 0.0`）时，优先将四路电机整体下移/上移，**完整保留电机之间的差动姿态控制力矩**，防止单侧电机削顶导致空中姿态失控翻转。

### 5.5 输出：LEDC PWM

- 频率 **20kHz**（高于人耳，避免啸叫），分辨率 10bit（0–1023）；
- 每路独立占空比，输出前经 `safety_apply_motor_gate()` 总闸门把关。

### 5.6 安全逻辑（五态状态机，`safety.h`）

| 保护 | 触发 | 动作 |
|------|------|------|
| 解锁四重联锁 | `LOCKED` 态请求解锁 | 必须同时满足：未失联 (`≤500ms`) + 机身平放 (`≤60°`) + 电压 `≥3.5V` + 油门 `≤5%` |
| 失联保护 | `>500ms` 无有效控制帧 | 切入 `FLIGHT_STATE_FAILSAFE`（Sticky 锁死），油门归零，重连后不自动复转，需拨回上锁复位 |
| 倾角翻机保护 | `\|roll\|` 或 `\|pitch\| >60°` | 立即切入 `FLIGHT_STATE_FAILSAFE` 切断电机 |
| 一级低压告警 | 电池 `<3.5V`（带载） | 切入 `FLIGHT_STATE_LOWBAT_WARN`，地面站告警，保留操控权以便飞回 |
| 二级严重低压 | 电池 `<3.3V` | 切入 `FLIGHT_STATE_LOWBAT_LAND`，强制姿态回平且限制最大油门 `≤25%` 缓降 |
| 电机总闸门 | `safety_apply_motor_gate` | 只要处于 `LOCKED` 或 `FAILSAFE`，混控后无条件将四路电机焊死为 `0.0f` |
| 物理急停 | 地面站 `KILL` 键 / 拔电池 | 连发 3 帧硬件级 `PROTOCOL_FLAG_KILL` 强制锁死 / 物理断电 |

---

## 6. FreeRTOS 任务规划（双核）

| 任务 | 核 | 频率 | 职责 |
|------|----|------|------|
| `vControlTask` | **Core1** | **400Hz** | `ble_comm_get_command` (`<1µs`) → 读 IMU → LPF → Mahony → `safety_update` → 串级 PID → `mixer_quad_x` → `safety_apply_motor_gate` → PWM |
| （外环，在控制任务内分频） | Core1 | **100Hz** | 每 4 拍执行一次角度外环 |
| BLE GATT 回调 + `vTelemTask` | **Core0** | **50Hz 下行 / 20Hz 上行** | BLE Write-No-Resp 接收 13B 控制帧写入 `portMUX`；20Hz Notify 推送 19B 遥测帧 |
| `vLedTask` | Core0 | 2Hz | 状态灯指示 |

---

## 7. 调试、测试与调参

- **桌面侧单元测试**：
  - C 端：`firmware/test/` 包含 LPF / Mahony / PID / 混控 / 安全状态机 / 二进制协议共 **6 套单元测试**，喂合成数据断言输出，**不烧板即可 100% 验证核心飞控逻辑**。
  - 前端：`controller/test/verify_protocol.mjs` 验证死区/Expo 数学、13B/19B 字节序与 CRC-8 对齐；`controller/` 内置离线飞控仿真器（`Simulator`），无硬件即可演练完整解锁/告警/急停闭环。
- **调参顺序**：先内环（角速度）→ 再外环（角度）→ 最后 yaw；
  先 roll/pitch → 后 yaw；先小增益 → 逐步加大到临界振荡再回退 30–50%。
- **安全调参**：先无桨 → 单轴测试架 → 系绳 → 低空。

---

## 8. 本期范围 vs 后续扩展

| 本期（阶段1） | 后续扩展（预留接口/另行加装） |
|---------------|------------------------------|
| MPU6050 姿态 + Mahony + 串级 PID + Quad-X | BMP280 / VL53L1X 定高 |
| QMC5883P 航向锁定 | 光流定点 / 定距移动（需位置环） |
| BLE 50Hz 低延迟手柄/地面站手动控制 | 自动航线 / 返航 / 在线 PID 参数无线烧录 |
| 短时小范围悬停（允许漂移） | 高精度定点（UWB / 光流） |

---

## 9. 全流程

### 9.1 硬件选型与 BOM

> 详细清单见 **`docs/BOM.md`**（必购 ≈¥78，板免费 ≈¥70）。

### 9.2 引脚分配（ESP32-S3-N16R8，最终以原理图为准）

| 信号 | GPIO | 外设 | 说明 |
|------|------|------|------|
| I²C | GPIO8 / GPIO9 | SDA / SCL | 400kHz，MPU6050 + QMC5883P |
| M1 | GPIO1 | LEDC CH0 | 右前，20kHz |
| M2 | GPIO2 | LEDC CH1 | 右后 |
| M3 | GPIO3 | LEDC CH2 | 左后 |
| M4 | GPIO4 | LEDC CH3 | 左前 |
| VBAT | GPIO5 | ADC1_CH4 | 100k/100k 分压 + 100nF |
| 解锁 | GPIO10 | GPIO 输入 | 拨动开关（可选） |
| 状态灯 | GPIO48 | WS2812 | 板载 |
| USB | GPIO19/20 | 原生 USB | **勿占用** |
| 调试 | GPIO43/44 | UART0 | USB 串口 |

**注意**：N16R8 的 **PSRAM 占用 GPIO35–37**、Flash 占用 GPIO26–32，**都不可使用**；
不同厂家 N16R8 开发板的排针顺序不同，**画原理图前必须逐脚核对实物丝印**。
磁力计 **QMC5883P 的 I²C 地址是 0x2C**（不是 L 版的 0x0D），驱动见 `firmware/src/drivers/qmc5883p.*`；
两者**寄存器映射不兼容，不能套用 QMC5883L 库**。

**尺寸与重量**：N16R8 约 25.5×52.5mm（带针约 10g），比 50×50 中心岛**每边探出约 1.25mm**，可接受；
若要完全包住，把中心岛放大到 **54×54**。整机估重约 67g，推重比约 2.1，仍够飞。

> **电机安装（重要）**：8520 本体约 φ8.5mm，机架需 **φ8.6 压入孔 + 出线槽**；
> 现有 v8 板框的 φ14.2 电机位是按更大外径画的，**打样前必须改**（WORKPLAN P1-2）。

### 9.3 电源树

```
1S 3.7V ─┬─ 4× 8520 电机（经 AO3400 低边开关）
         ├─ MT3608 ──▶ 5V ──▶ ESP32 5V ──▶ 板载 LDO ──▶ 3.3V（MPU/mag）
         └─ 100k/100k 分压 ──▶ GPIO5（VBAT 采样，100nF 滤波）
```

- 5V 端并 **470µF + 10µF**；
- 3.3V 端并 **10µF + 0.1µF**；
- 电池插头选 PH2.0，**可快速拔断**（物理急停）。

### 9.4 首飞检查清单

- [ ] 拆桨上电，地面站看姿态是否平稳（不抖、不发散）。
- [ ] 电机转向正确（Quad-X 对角同向、相邻反向）。
- [ ] 解锁四重联锁就位，未解锁或晃动机身时电机绝不误转。
- [ ] 失联 500ms 自动切入 `FAILSAFE` 锁死（断 BLE / 关页面实测）。
- [ ] 倾角 >60° 停机锁死（手动大幅倾斜实测）。
- [ ] 系绳或手托低油门，确认姿态回正。
- [ ] 电池电压显示正常、3.5V 告警与 3.3V 缓降有效。

### 9.5 本期范围边界（阶段1）

> 阶段1 = **姿态自稳 + 航向锁定 + 手动飞行 + 短时小范围悬停（允许漂移）**。
> **不做**定高、定点、定距移动、自主航线（需光流/测距 + 位置环，见 §8）。

---

## 10. 决策记录 + 设计问题表

| # | 问题 | 结论 |
|---|------|------|
| Q1 | ESP32 性能够吗？ | 双核 240MHz，400Hz 控制 + BLE 足够；**不建议单核 C3**（射频与控制抢核，抖动大）。 |
| Q2 | 为什么用有刷？ | 便宜、驱动简单、1S 安全；代价是寿命短、效率低、抗风差。 |
| Q3 | 有刷 PWM 用多少？ | **20kHz / 10bit**（LEDC）。**不是**舵机 50Hz；低频会啸叫、抖动。 |
| Q4 | 1S 怎么给 ESP32 供电？ | 必须 **MT3608 升 5V**；AMS1117 从 3.7V 降压压差不够。 |
| Q5 | 磁力计能定点吗？ | **不能**。它只锁航向；水平位置仍会漂，定点需要光流/GPS。 |
| Q6 | 控制频率多少合适？ | **400 / 100 / 50Hz**（内环/外环/BLE 下行）。500Hz+ 收益小。 |
| Q7 | 安全怎么做？ | 五态状态机（失联 500ms、倾角 >60° Sticky 锁死、3.5V/3.3V 分级低压、解锁四重联锁、电机总闸门）+ 物理急停。 |
| Q8 | 8520 怎么固定？ | PCB 做 **φ8.6 压入孔 + 出线槽**；现有 v8 的 φ14.2 要改。 |
| Q9 | 为什么用 BLE GATT 替代 WiFi JSON？ | BLE Write-No-Resp 支持 7.5–15ms 连接间隔，13B 二进制帧无需分包、无 TCP 队头阻塞，且射频峰值功耗远低于 WiFi。 |
| Q10 | 能定距移动吗？ | **不能**。需光流 + 测距 + 位置环，属后续扩展。 |

---

## 11. 下一步

> 当前推进状态与逐条待办见 **`docs/WORKPLAN.md`**（§0 进度看板 / §14 下一步 TOP 3）。
