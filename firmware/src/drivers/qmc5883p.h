#pragma once

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// QMC5883P 三轴磁力计驱动
//   - I²C 7 位地址：0x2C（写 0x58 / 读 0x59）
//   - 注意：与 QMC5883L（0x0D）寄存器映射完全不同，
//           不能套用 QMC5883L / HMC5883L 的库。
//   数据手册：QST QMC5883P（CHIPID = 0x80）
// ============================================================

#define QMC5883P_ADDR          0x2C
#define QMC5883P_CHIP_ID       0x80

// 寄存器
#define QMC5883P_REG_CHIPID    0x00
#define QMC5883P_REG_XOUT_LSB  0x01
#define QMC5883P_REG_XOUT_MSB  0x02
#define QMC5883P_REG_YOUT_LSB  0x03
#define QMC5883P_REG_YOUT_MSB  0x04
#define QMC5883P_REG_ZOUT_LSB  0x05
#define QMC5883P_REG_ZOUT_MSB  0x06
#define QMC5883P_REG_STATUS    0x09
#define QMC5883P_REG_CONTROL1  0x0A
#define QMC5883P_REG_CONTROL2  0x0B

// STATUS 位
#define QMC5883P_STATUS_DRDY   0x01   // 数据就绪
#define QMC5883P_STATUS_OVFL   0x02   // 溢出（量程不够）

// 量程：CONTROL2[3:2]
typedef enum {
    QMC5883P_RANGE_30G = 0x00,   // 1000   LSB/G
    QMC5883P_RANGE_12G = 0x01,   // 2500   LSB/G
    QMC5883P_RANGE_8G  = 0x02,   // 3750   LSB/G（默认）
    QMC5883P_RANGE_2G  = 0x03,   // 15000  LSB/G
} qmc5883p_range_t;

// 输出数据率：CONTROL1[3:2]
typedef enum {
    QMC5883P_ODR_10HZ  = 0x00,
    QMC5883P_ODR_50HZ  = 0x01,
    QMC5883P_ODR_100HZ = 0x02,
    QMC5883P_ODR_200HZ = 0x03,
} qmc5883p_odr_t;

// 过采样：CONTROL1[5:4]
typedef enum {
    QMC5883P_OSR_8 = 0x00,
    QMC5883P_OSR_4 = 0x01,
    QMC5883P_OSR_2 = 0x02,
    QMC5883P_OSR_1 = 0x03,
} qmc5883p_osr_t;

// 工作模式：CONTROL1[1:0]
typedef enum {
    QMC5883P_MODE_SUSPEND    = 0x00,
    QMC5883P_MODE_NORMAL     = 0x01,
    QMC5883P_MODE_SINGLE     = 0x02,
    QMC5883P_MODE_CONTINUOUS = 0x03,
} qmc5883p_mode_t;

// 初始化（软复位 -> 校验 0x80 -> 配置 ±8G / 100Hz / OSR8 / 连续）
bool qmc5883p_init(TwoWire &wire = Wire);

// 单独配置量程 / ODR / OSR（默认连续模式，Set-Reset ON）
bool qmc5883p_configure(qmc5883p_range_t range,
                        qmc5883p_odr_t odr,
                        qmc5883p_osr_t osr,
                        TwoWire &wire = Wire);

uint8_t qmc5883p_chip_id(TwoWire &wire = Wire);
bool    qmc5883p_data_ready(TwoWire &wire = Wire);

// 原始值（int16，小端，2 的补码）
bool qmc5883p_read_raw(int16_t out[3], TwoWire &wire = Wire);

// 高斯值（已减去硬铁偏移）
bool qmc5883p_read_gauss(float out[3], TwoWire &wire = Wire);

// 航向角（0~360°，机头方向需按实际安装调整符号）
bool qmc5883p_read_heading_deg(float *heading_deg, TwoWire &wire = Wire);

// 硬铁校准偏移（静止旋转采集到的 min/max 中点）
void qmc5883p_set_hard_iron(float ox, float oy, float oz);

// 当前量程对应的 LSB/G
float qmc5883p_lsb_per_gauss(void);
