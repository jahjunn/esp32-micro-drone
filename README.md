# 轻量无人机（ESP32 + 有刷 8520）

一个自制的微型四轴飞控学习项目：**板即机架**（嘉立创异形 PCB 当机架），
**ESP32-S3** 主控，**MPU6050 + QMC5883P**，**8520 空心杯**，**BLE 蓝牙低延迟手机/手柄控制**。

> 目标：**能姿态自稳、航向锁定、手动飞行、短时小范围悬停**（允许漂移）。
> 不追求高精度定点/定高，先把"能大致飞"跑通。

## 能力边界

| 能力 | 是否支持 |
|------|----------|
| 姿态自稳（不翻） | ✅ |
| 航向锁定（磁力计） | ✅ |
| BLE 手动飞行（10–20m，50Hz 低延迟） | ✅ |
| 短时小范围悬停（会缓慢漂移） | ✅ |
| 定高 / 定点 / 定距移动 | ❌（P8 扩展） |
| 抗风 / 自动航线 | ❌ |

## 硬件方案

| 部件 | 选型 |
|------|------|
| 主控 | ESP32-S3-N16R8 开发板（双核 240MHz，16MB/8MB，USB-C） |
| 姿态传感器 | MPU6050 GY-521（I²C） |
| 磁力计 | QMC5883P（I²C 0x2C） |
| 电机 | 8520 空心杯 ×4（1S） |
| 驱动 | AO3400（N-MOS）×4 + SS34 续流 ×4 |
| 电池 | 1S 3.7V 600mAh ≥25C |
| 电源 | 1S → MT3608 升 5V → ESP32；VBAT 分压进 ADC |
| 机架 | 嘉立创异形 PCB（100mm 对角轴距） |
| 控制 | ESP32-S3 BLE GATT（7.5–15ms 连接间隔）+ 跨平台手柄/地面站（Web Bluetooth / Gamepad / Capacitor App） |

**成本**：必购 ≈¥78（PCB 用打样券 ≈¥70）。

## 软件架构

- **ESP32-S3 + Arduino/ESP-IDF + FreeRTOS（双核）**
  - Core1：400Hz 实时控制（读 IMU → LPF → Mahony → 串级 PID → Quad-X → PWM）
  - Core0：BLE GATT 50Hz 控制下行 / 10–20Hz 遥测上行（`portMUX` 跨核零积压同步）、安全监控、状态灯
- **通信协议**：13B 下行控制帧（`0xAA 0x55`）+ 19B 上行遥测帧（`0x55 0xAA`），小端序紧凑二进制 + CRC-8 校验
- **算法**：二阶巴特沃斯 LPF（30–50Hz）+ Mahony 四元数 + 串级 PID + 保均值抗饱和 Quad-X 混控
- **输出**：LEDC 4× PWM，20kHz / 10bit
- **安全**：五态安全状态机（失联 500ms / 倾角 >60° 触发 Sticky Failsafe 锁死、3.5V 一级低压告警、3.3V 二级低压自动回平缓降、解锁四重联锁、电机总闸门 + 物理急停）

## 目录结构

```
轻量无人机/
├── docs/                  # 文档
│   ├── ARCHITECTURE.md    # 架构与算法
│   ├── DESIGN_FLOW.md     # 设计路径
│   ├── WORKPLAN.md        # 工作计划（工作项/验收/依赖）
│   └── BOM.md             # 物料清单
├── hardware/              # 板框 DXF 与生成脚本（v1–v8）
├── firmware/              # 机载固件（ESP32-S3 工程、驱动、控制算法、BLE 通信与 6 套 C 桌面单测）
│   ├── platformio.ini     # N16R8 编译与分区配置
│   ├── src/
│   │   ├── drivers/       # 硬件驱动（qmc5883p 等）
│   │   ├── flight/        # 飞控算法（types / biquad_lpf / mahony / pid / mixer / safety）
│   │   └── comm/          # BLE GATT 服务端与二进制协议（protocol.h / ble_server.*）
│   └── test/              # 桌面侧单元测试（LPF / Mahony / PID / Mixer / Safety / Protocol）
└── controller/            # 跨平台低延迟蓝牙手柄与地面站（Web Bluetooth + Gamepad + Capacitor + 离线飞控仿真器）
```

## 快速开始

> 硬件（P1）与软件（P2–P6）当前并行推进中；算法、BLE 协议与地面站已在桌面端完成闭环验证。

1. 阅读 `docs/ARCHITECTURE.md` 了解方案；
2. 按 `docs/BOM.md` 采购；
3. 按 `docs/WORKPLAN.md` 逐项推进（每项带验收标准）；
4. 体验地面站与离线飞控仿真：`node controller/test/verify_protocol.mjs && node controller/dev_server.mjs`。

## 安全红线

- 非飞行测试**一律拆桨**；
- 首次飞行必须**系绳或单轴测试架**；
- 上电默认锁定，解锁后先低油门；
- failsafe 每次飞行前实测；
- 人与桨盘面保持距离。

## 当前进度

| 阶段 | 状态 |
|------|------|
| P0 选型/BOM | ✅ 完成 |
| P1 硬件 | 🟡 板框 v8 已有，**电机位待改配 8520** |
| P2 软件环境 | 🟡 `platformio.ini`（N16R8）、`firmware/` 与 `controller/` 工程骨架及双端桌面单测环境已就绪；外设主入口待建 |
| P3 驱动与通信 | 🟡 `qmc5883p` 磁力计驱动、`ble_server` BLE GATT 服务端、`protocol.h` 二进制协议（13B/19B + CRC-8）已完成；MPU6050 / PWM / VBAT / 状态灯待开始 |
| P4 算法+单测 | 🟡 `types.h`、二阶低通滤波 `biquad_lpf.h`、Mahony `mahony.h`、PID `pid.h`、抗饱和混控 `mixer.h`、五态安全机 `safety.h` 及 6 套 C 桌面单测已完成；传感器标定待推进 |
| P5 双核编排 | ⬜ 待外设驱动合拢后接入 `main.cpp` |
| P6 调试与地面站 | 🟡 `controller/` 蓝牙手柄与地面站（Mode 2 双摇杆、Gamepad、死区/Expo/Trim、安全联锁、KILL 急停、遥测看板、离线仿真器）已完成；实机台架调参待开始 |
| P7–P9 | ⬜ 未开始 |

详见 `docs/WORKPLAN.md`。
