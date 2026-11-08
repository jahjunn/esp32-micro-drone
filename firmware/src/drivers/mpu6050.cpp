#include "mpu6050.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ±2000 °/s (FS_SEL = 3): 16.4 LSB / (°/s) -> rad/s
static constexpr float MPU_GYRO_SCALE_RADS  = ((float)M_PI / 180.0f) / 16.4f;
// ±8g (AFS_SEL = 2): 4096 LSB / g（相比 ±2g/±4g 抗有刷高频振动削顶）
static constexpr float MPU_ACCEL_SCALE_G    = 1.0f / 4096.0f;

static bool write_reg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
    bus.beginTransmission(addr);
    bus.write(reg);
    bus.write(val);
    return bus.endTransmission() == 0;
}

static bool read_reg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *out_val) {
    bus.beginTransmission(addr);
    bus.write(reg);
    if (bus.endTransmission(false) != 0) {
        return false;
    }
    if (bus.requestFrom((int)addr, 1) != 1) {
        return false;
    }
    *out_val = (uint8_t)bus.read();
    return true;
}

bool mpu6050_init(TwoWire &bus, uint8_t addr) {
    delay(40); // 上电等待 MPU6050 内部稳压与振荡器稳定

    uint8_t who = 0U;
    bool probed = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (read_reg(bus, addr, MPU6050_REG_WHO_AM_I, &who)) {
            probed = true;
            break;
        }
        delay(20);
    }
    // 标准 MPU6050 返回 0x68（部分兼容/AD0 丝印差异芯片最低位固定或报告 0x68/0x98）
    if (!probed || (who != MPU6050_WHO_AM_I_VAL && who != 0x69U)) {
        return false;
    }

    // 1. 触发片内寄存器全复位
    if (!write_reg(bus, addr, MPU6050_REG_PWR_MGMT_1, 0x80U)) {
        return false;
    }
    delay(50);

    // 2. 唤醒并选择 PLL X-Gyro 高稳时钟源 (CLKSEL = 0x01)
    if (!write_reg(bus, addr, MPU6050_REG_PWR_MGMT_1, 0x01U)) {
        return false;
    }
    delay(10);

    // 3. 使能硬件 DLPF_CFG = 2（Accel 94Hz / Gyro 98Hz，内部采样率 1kHz），抑制 >200Hz 混叠噪声
    if (!write_reg(bus, addr, MPU6050_REG_CONFIG, 0x02U)) {
        return false;
    }

    // 4. 采样分频 SMPLRT_DIV = 0 -> 1kHz 刷新，保证 Core1 400Hz 轮询每次均读到新鲜样本
    if (!write_reg(bus, addr, MPU6050_REG_SMPLRT_DIV, 0x00U)) {
        return false;
    }

    // 5. 陀螺仪满量程 ±2000 deg/s (FS_SEL = 3 -> 0x18)
    if (!write_reg(bus, addr, MPU6050_REG_GYRO_CONFIG, 0x18U)) {
        return false;
    }

    // 6. 加速度计满量程 ±8g (AFS_SEL = 2 -> 0x10)
    if (!write_reg(bus, addr, MPU6050_REG_ACCEL_CONFIG, 0x10U)) {
        return false;
    }

    return true;
}

bool mpu6050_read(Vector3_t *gyro_flu, Vector3_t *accel_flu, TwoWire &bus, uint8_t addr) {
    if (gyro_flu == nullptr || accel_flu == nullptr) {
        return false;
    }

    bus.beginTransmission(addr);
    bus.write(MPU6050_REG_ACCEL_XOUT_H);
    if (bus.endTransmission(false) != 0) {
        return false;
    }

    constexpr int kBurstLen = 14;
    if (bus.requestFrom((int)addr, kBurstLen) != kBurstLen) {
        return false;
    }

    uint8_t raw[kBurstLen];
    for (int i = 0; i < kBurstLen; ++i) {
        int b = bus.read();
        if (b < 0) {
            return false;
        }
        raw[i] = (uint8_t)b;
    }

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    // raw[6..7] 为芯片温度 TEMP_OUT，暂不使用
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    accel_flu->x = (float)ax * MPU_ACCEL_SCALE_G;
    accel_flu->y = (float)ay * MPU_ACCEL_SCALE_G;
    accel_flu->z = (float)az * MPU_ACCEL_SCALE_G;

    gyro_flu->x  = (float)gx * MPU_GYRO_SCALE_RADS;
    gyro_flu->y  = (float)gy * MPU_GYRO_SCALE_RADS;
    gyro_flu->z  = (float)gz * MPU_GYRO_SCALE_RADS;

    return true;
}
