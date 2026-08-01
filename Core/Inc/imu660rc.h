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
#define IMU660RC_OUTX_L_A           ( 0x28 )
#define IMU660RC_OUTX_H_A           ( 0x29 )

/* ========== 滤波参数 ========== */
#define IMU660RC_FILTER_ALPHA    0.06f  /* 低通: 快速跟踪方向反转 (~3ms) */
#define IMU660RC_DEADBAND        0.003f  /* 死区阈值 (g) */
#define IMU660RC_CALIB_SAMPLES   300     /* 零偏校准采样数 */

/* ========== API ========== */
bool imu660rc_init(void);
float imu660rc_read_acc_y(void);
float imu660rc_get_acc_y_filtered(void);  /* 滤波+零偏校准后的 Y 轴加速度 (g) */
void imu660rc_calibrate(void);             /* 零偏校准 (静止状态下调用) */
bool imu660rc_is_calibrated(void);

#endif
