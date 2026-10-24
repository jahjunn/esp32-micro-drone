#include "qmc5883p.h"

#include <math.h>

// ============================================================
// QMC5883P 驱动实现
//   寄存器映射与 QMC5883L 不同，切勿混用。
//   CONTROL1 (0x0A)：DSR[7:6] | OSR[5:4] | ODR[3:2] | MODE[1:0]
//   CONTROL2 (0x0B)：SRST[7] | ST[6] | - | RNG[3:2] | SET/RESET[1:0]
// ============================================================

static TwoWire *s_wire = &Wire;
static float    s_hard_iron[3] = {0.0f, 0.0f, 0.0f};
static float    s_lsb_per_g = 3750.0f;   // 默认 ±8G
static bool     s_inited = false;

static bool reg_write(uint8_t reg, uint8_t val) {
    s_wire->beginTransmission(QMC5883P_ADDR);
    s_wire->write(reg);
    s_wire->write(val);
    return s_wire->endTransmission() == 0;
}

static bool reg_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    s_wire->beginTransmission(QMC5883P_ADDR);
    s_wire->write(reg);
    if (s_wire->endTransmission(false) != 0) {
        return false;
    }
    if (s_wire->requestFrom((uint8_t)QMC5883P_ADDR, (uint8_t)len) != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = s_wire->read();
    }
    return true;
}

float qmc5883p_lsb_per_gauss(void) {
    return s_lsb_per_g;
}

void qmc5883p_set_hard_iron(float ox, float oy, float oz) {
    s_hard_iron[0] = ox;
    s_hard_iron[1] = oy;
    s_hard_iron[2] = oz;
}

uint8_t qmc5883p_chip_id(TwoWire &wire) {
    s_wire = &wire;
    uint8_t id = 0;
    if (!reg_read(QMC5883P_REG_CHIPID, &id, 1)) {
        return 0;
    }
    return id;
}

bool qmc5883p_configure(qmc5883p_range_t range,
                        qmc5883p_odr_t odr,
                        qmc5883p_osr_t osr,
                        TwoWire &wire) {
    s_wire = &wire;

    // CONTROL1：DSR=1x(00) | OSR | ODR | 连续模式(11)
    uint8_t c1 = (uint8_t)(((uint8_t)osr << 4) |
                           ((uint8_t)odr << 2) |
                           (uint8_t)QMC5883P_MODE_CONTINUOUS);

    // CONTROL2：不复位/不自检 | 量程 | Set-Reset ON(00)
    uint8_t c2 = (uint8_t)(((uint8_t)range << 2) | 0x00);

    if (!reg_write(QMC5883P_REG_CONTROL1, c1)) {
        return false;
    }
    if (!reg_write(QMC5883P_REG_CONTROL2, c2)) {
        return false;
    }

    switch (range) {
        case QMC5883P_RANGE_30G: s_lsb_per_g = 1000.0f;  break;
        case QMC5883P_RANGE_12G: s_lsb_per_g = 2500.0f;  break;
        case QMC5883P_RANGE_8G:  s_lsb_per_g = 3750.0f;  break;
        case QMC5883P_RANGE_2G:  s_lsb_per_g = 15000.0f; break;
        default:                 s_lsb_per_g = 3750.0f;  break;
    }

    delay(10);   // 等第一帧数据
    return true;
}

bool qmc5883p_init(TwoWire &wire) {
    s_wire = &wire;
    s_inited = false;

    // 1) 软复位（CONTROL2 bit7），等 50ms
    if (!reg_write(QMC5883P_REG_CONTROL2, 0x80)) {
        return false;
    }
    delay(50);

    // 2) 校验芯片 ID（QMC5883P = 0x80）
    uint8_t id = 0;
    if (!reg_read(QMC5883P_REG_CHIPID, &id, 1)) {
        return false;
    }
    if (id != QMC5883P_CHIP_ID) {
        return false;
    }

    // 3) 默认配置：±8G / 100Hz / OSR 8x / 连续
    if (!qmc5883p_configure(QMC5883P_RANGE_8G,
                            QMC5883P_ODR_100HZ,
                            QMC5883P_OSR_8,
                            wire)) {
        return false;
    }

    s_inited = true;
    return true;
}

bool qmc5883p_data_ready(TwoWire &wire) {
    s_wire = &wire;
    uint8_t st = 0;
    if (!reg_read(QMC5883P_REG_STATUS, &st, 1)) {
        return false;
    }
    return (st & QMC5883P_STATUS_DRDY) != 0;
}

bool qmc5883p_read_raw(int16_t out[3], TwoWire &wire) {
    s_wire = &wire;

    uint8_t buf[6] = {0};
    // 从 XOUT_LSB(0x01) 连续读 6 字节：X/Y/Z 各 16 位，小端
    if (!reg_read(QMC5883P_REG_XOUT_LSB, buf, 6)) {
        return false;
    }

    out[0] = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    out[1] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    out[2] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    return true;
}

bool qmc5883p_read_gauss(float out[3], TwoWire &wire) {
    int16_t raw[3];
    if (!qmc5883p_read_raw(raw, wire)) {
        return false;
    }

    out[0] = (float)raw[0] / s_lsb_per_g - s_hard_iron[0];
    out[1] = (float)raw[1] / s_lsb_per_g - s_hard_iron[1];
    out[2] = (float)raw[2] / s_lsb_per_g - s_hard_iron[2];
    return true;
}

bool qmc5883p_read_heading_deg(float *heading_deg, TwoWire &wire) {
    float g[3];
    if (!qmc5883p_read_gauss(g, wire)) {
        return false;
    }

    // 水平放置时用 X/Y 计算航向；机头方向按实际安装调整符号
    float heading = atan2f(-g[1], g[0]) * (180.0f / (float)M_PI);
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    *heading_deg = heading;
    return true;
}
