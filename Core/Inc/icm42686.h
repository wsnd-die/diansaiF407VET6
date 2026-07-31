/**
 * @file    icm42686.h
 * @brief   ICM42686-P 6轴IMU (I2C) — 挂在 OLED 的 I2C2 总线上
 * @note    I2C2: PB10=SCL, PB11=SDA, 100kHz
 *          AD0=0 → 地址 0x68
 */

#ifndef ICM42686_H_
#define ICM42686_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== I2C 地址 ========== */
#define ICM42686_I2C_ADDR       0x69U   /* AD0=VDDIO, 扫描确认 */

/* ========== 灵敏度 ========== */
#define SENS_ICM42686_GYRO_2000DPS   16.4f
#define SENS_ICM42686_GYRO_1000DPS   32.8f
#define SENS_ICM42686_GYRO_500DPS    65.5f
#define SENS_ICM42686_GYRO_250DPS    131.0f
#define SENS_ICM42686_GYRO_125DPS    262.0f
#define SENS_ICM42686_GYRO_62_5DPS   524.3f
#define SENS_ICM42686_GYRO_31_25DPS  1048.6f

#define SENS_ICM42686_ACC_16G  2048
#define SENS_ICM42686_ACC_8G   4096
#define SENS_ICM42686_ACC_4G   8192
#define SENS_ICM42686_ACC_2G   16384

#define ICM42686_WHO_AM_I_VAL  0x44

/* ========== 枚举 ========== */
typedef enum {
    GYRO_2000DPS = 0, GYRO_1000DPS, GYRO_500DPS, GYRO_250DPS,
    GYRO_125DPS, GYRO_62_5DPS, GYRO_31_25DPS,
} ICM42686_GyroFSR;

typedef enum {
    GYRO_ODR_12_5HZ = 0, GYRO_ODR_25HZ, GYRO_ODR_50HZ, GYRO_ODR_100HZ,
    GYRO_ODR_200HZ, GYRO_ODR_500HZ, GYRO_ODR_1000HZ, GYRO_ODR_2000HZ,
    GYRO_ODR_4000HZ, GYRO_ODR_8000HZ, GYRO_ODR_16000HZ, GYRO_ODR_32000HZ,
} ICM42686_GyroODR;

typedef enum { ACC_16G = 0, ACC_8G, ACC_4G, ACC_2G } ICM42686_AccFSR;

typedef enum {
    ACC_ODR_12_5HZ = 0, ACC_ODR_25HZ, ACC_ODR_50HZ, ACC_ODR_100HZ,
    ACC_ODR_200HZ, ACC_ODR_500HZ, ACC_ODR_1000HZ, ACC_ODR_2000HZ,
    ACC_ODR_4000HZ, ACC_ODR_8000HZ, ACC_ODR_16000HZ, ACC_ODR_32000HZ,
} ICM42686_AccODR;

typedef enum { FILTER_1ST = 0, FILTER_2ST, FILTER_3ST } ICM42686_FilterOrder;

typedef enum {
    BW_FACTOR_2 = 0, BW_FACTOR_4, BW_FACTOR_5, BW_FACTOR_8,
    BW_FACTOR_10, BW_FACTOR_16, BW_FACTOR_20, BW_FACTOR_40,
    BW_LOW_LATENCY_1, BW_LOW_LATENCY_2,
} ICM42686_BandwidthFactor;

/* ========== 数据结构 ========== */
typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
} ICM42686_RawData;

typedef struct {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} ICM42686_RealData;

typedef struct {
    ICM42686_GyroFSR         gyro_fsr;
    ICM42686_GyroODR         gyro_odr;
    ICM42686_AccFSR          acc_fsr;
    ICM42686_AccODR          acc_odr;
    ICM42686_FilterOrder     gyro_filter_order;
    ICM42686_BandwidthFactor gyro_bw;
    ICM42686_FilterOrder     acc_filter_order;
    ICM42686_BandwidthFactor acc_bw;
} ICM42686_Config;

/* ========== 全局变量 ========== */
extern ICM42686_RealData g_icm42686;
extern ICM42686_RawData  g_icm42686_raw;
extern bool g_icm42686_ready;

/* ========== API ========== */
HAL_StatusTypeDef ICM42686_Init(const ICM42686_Config *config);
void ICM42686_ReadData(void);
float ICM42686_GetAccelY(void);

#endif
