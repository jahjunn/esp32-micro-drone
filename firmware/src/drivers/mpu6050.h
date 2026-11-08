#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stdbool.h>
#include "../flight/types.h"

/*
 * MPU6050 6 轴惯性测量单元驱动 (I2C)
 *
 * 【坐标系说明】：
 *   - 输出为正面朝上贴装的芯片原生坐标系 FLU（+X 前、+Y 左、+Z 上，静止时 Z 轴约 +1.0g）；
 *   - 上层控制环路必须调用 controller_map_flu_to_frd() 转为飞控 FRD 右手系，禁止裸轴直传。
 */

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

#define MPU6050_WHO_AM_I_VAL      0x68

// 初始化 MPU6050：复位芯片、切换 PLL 时钟、启用 94Hz 硬件抗混叠 DLPF、配置 ±2000°/s & ±8g
// 返回 true 表示 WHO_AM_I 校验与寄存器写入全部成功；需先完成 Wire.begin()
bool mpu6050_init(TwoWire &bus = Wire, uint8_t addr = 0x68);

// 14 字节突发读取原始惯性量，并换算为 FLU 物理单位：
//   - gyro_flu  [rad/s]
//   - accel_flu [g]
// 返回 false 表示本次 I2C 读取异常（上层应拦截并交由安全防火墙处理）
bool mpu6050_read(Vector3_t *gyro_flu, Vector3_t *accel_flu, TwoWire &bus = Wire, uint8_t addr = 0x68);
