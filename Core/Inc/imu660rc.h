/**
 * @file    imu660rc.h
 * @brief   IMU660RC (ICM-42688-P) 精简驱动 — 仅 Y 轴加速度
 * @note    软件 I2C: PB4=SDA, PB5=SCL, 地址 0x6B
 */

#ifndef IMU660RC_H_
#define IMU660RC_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== I2C 地址 ========== */
#define IMU660RC_I2C_ADDR       0x6BU   /* SA0 拉高 (模块默认) */
#define IMU660RC_I2C_ADDR_ALT   0x6AU   /* SA0 接地 */

/* ========== 灵敏度 (LSB/g) ========== */
#define IMU660RC_ACC_SENS_2G    16384.0f

/* ========== 寄存器 ========== */
#define IMU660RC_CHIP_ID        0x0FU
#define IMU660RC_CHIP_ID_VAL    0x70U

#define IMU660RC_FUNC_CFG_ACCESS 0x01U
#define IMU660RC_CTRL1           0x10U
#define IMU660RC_CTRL3           0x12U
#define IMU660RC_CTRL8           0x17U
#define IMU660RC_STATUS_REG      0x1EU

#define IMU660RC_OUTY_L_A       0x2AU
#define IMU660RC_OUTY_H_A       0x2BU

/* ========== API ========== */
bool imu660rc_init(void);
float imu660rc_read_acc_y(void);

#endif
