/**
 * @file    icm42686.c
 * @brief   ICM42686-P 6轴IMU (软件I2C, PB4=SDA, PB5=SCL)
 * @note    使用 HAL_GPIO_WritePin/ReadPin, 时序与已验证的 OLED 驱动一致
 */

#include "icm42686.h"
#include "app.h"

/* ========== GPIO 操作 (与 OLED 完全一致的方式) ========== */
#define SCL_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define SDA_RD()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)

/* SDA 切换为输入 (上拉) 用于读数据 */
static void sda_input(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_4;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* SDA 切换为输出 (推挽) 用于写数据, 匹配原 OLED 驱动 */
static void sda_output(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_4;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* ========== 全局变量 ========== */
ICM42686_RealData g_icm42686 = {0};
ICM42686_RawData  g_icm42686_raw = {0};
bool g_icm42686_ready = false;

static float g_gyro_sens = SENS_ICM42686_GYRO_250DPS;
static float g_acc_sens  = SENS_ICM42686_ACC_2G;

/* I2C 延时 (~2us @168MHz, 对应 ~200kHz Fast-mode) */
static void i2c_delay(void)
{
    for (volatile int j = 0; j < 80; j++) {}
}

static void i2c_start(void)
{
    sda_output();
    SDA_H(); i2c_delay();
    SCL_H(); i2c_delay();
    SDA_L(); i2c_delay();
    SCL_L(); i2c_delay();
}

static void i2c_stop(void)
{
    sda_output();
    SDA_L(); i2c_delay();
    SCL_H(); i2c_delay();
    SDA_H(); i2c_delay();
}

/* 返回 0=ACK, 1=NACK */
static uint8_t i2c_wait_ack(void)
{
    uint8_t ack;
    sda_input();                 /* 释放 SDA */
    i2c_delay();
    SCL_H(); i2c_delay();
    ack = SDA_RD() ? 1U : 0U;
    SCL_L(); i2c_delay();
    sda_output();
    return ack;
}

static void i2c_write_byte(uint8_t data)
{
    sda_output();
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) SDA_H(); else SDA_L();
        i2c_delay();
        SCL_H(); i2c_delay();
        SCL_L(); i2c_delay();
        data <<= 1;
    }
    i2c_wait_ack();
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t data = 0;
    sda_input();
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        SCL_H(); i2c_delay();
        if (SDA_RD()) data |= 1;
        SCL_L(); i2c_delay();
    }
    sda_output();
    if (ack) SDA_L(); else SDA_H();   /* ACK / NACK */
    i2c_delay();
    SCL_H(); i2c_delay();
    SCL_L(); i2c_delay();
    SDA_H();
    return data;
}

static uint8_t g_icm_addr = 0x68U;  /* 运行时自动检测 */

/* ======================================================================
 *   寄存器操作 (使用自动检测的地址)
 * ====================================================================== */

static void icm_write_reg(uint8_t reg, uint8_t data)
{
    i2c_start();
    i2c_write_byte((uint8_t)(g_icm_addr << 1));
    i2c_write_byte(reg);
    i2c_write_byte(data);
    i2c_stop();
}

static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t val = 0;
    i2c_start();
    i2c_write_byte((uint8_t)(g_icm_addr << 1));
    i2c_write_byte(reg);
    i2c_start();
    i2c_write_byte((uint8_t)((g_icm_addr << 1) | 1U));
    val = i2c_read_byte(0);
    i2c_stop();
    return val;
}

static void icm_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_write_byte((uint8_t)(g_icm_addr << 1));
    i2c_write_byte(reg);
    i2c_start();
    i2c_write_byte((uint8_t)((g_icm_addr << 1) | 1U));
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2c_read_byte((uint8_t)(i < (len - 1) ? 1 : 0));
    }
    i2c_stop();
}

/* ======================================================================
 *   芯片识别 — 自动检测 I2C 地址 (0x68 / 0x69)
 * ====================================================================== */

static uint8_t _find_icm(void)
{
    uint8_t id;
    uint8_t found_addr = 0;
    uint16_t timeout;

    /* 步骤1: 扫描 0x68 和 0x69, 找 ACK */
    App_Uart6Printf("[ICM] Auto-detect addr:\r\n");
    for (uint8_t a = 0x68; a <= 0x69; a++) {
        uint8_t ack;
        sda_output();
        i2c_start();
        uint8_t d = (uint8_t)(a << 1);
        for (int i = 0; i < 8; i++) {
            if (d & 0x80) SDA_H(); else SDA_L();
            i2c_delay();
            SCL_H(); i2c_delay();
            SCL_L(); i2c_delay();
            d <<= 1;
        }
        sda_input();
        i2c_delay();
        SCL_H(); i2c_delay();
        ack = SDA_RD() ? 1U : 0U;
        SCL_L(); i2c_delay();
        i2c_stop();

        App_Uart6Printf("      0x%02X %s\r\n", a, ack == 0 ? "ACK" : "NACK");
        if (ack == 0) found_addr = a;
    }

    if (found_addr == 0) {
        g_icm_addr = 0;  /* 关键: 清零, 让 ICM42686_Init() 能正确检测失败 */
        App_Uart6Printf("[ICM42686] No device found!\r\n");
        return 0xFF;
    }

    g_icm_addr = found_addr;
    App_Uart6Printf("[ICM42686] Using addr=0x%02X\r\n", g_icm_addr);

    /* 步骤2: 读 WHO_AM_I */
    timeout = 0;
    do {
        id = icm_read_reg(0x75);
        App_Uart6Printf("[ICM42686] WHO_AM_I=0x%02X t=%d\r\n", id, timeout);
        if (id == ICM42686_WHO_AM_I_VAL) break;
        if (timeout++ >= 10) break;
        HAL_Delay(10);
    } while (1);

    if (ICM42686_WHO_AM_I_VAL == id) {
        App_Uart6Printf("[ICM42686] ID OK!\r\n");
    } else {
        App_Uart6Printf("[ICM42686] WARN: expected 0x%02X, got 0x%02X\r\n",
                        ICM42686_WHO_AM_I_VAL, id);
    }
    return id;
}

/* ======================================================================
 *   陀螺仪 / 加速度 / 滤波器配置 (略, 不变)
 * ====================================================================== */

static void _icm_set_gyro(ICM42686_GyroFSR fsr, ICM42686_GyroODR odr)
{
    uint8_t cfg = 0x00;
    switch (fsr) {
        case GYRO_31_25DPS: cfg = 0xC0; g_gyro_sens = SENS_ICM42686_GYRO_31_25DPS; break;
        case GYRO_62_5DPS:  cfg = 0xA0; g_gyro_sens = SENS_ICM42686_GYRO_62_5DPS;  break;
        case GYRO_125DPS:   cfg = 0x80; g_gyro_sens = SENS_ICM42686_GYRO_125DPS;   break;
        case GYRO_250DPS:   cfg = 0x60; g_gyro_sens = SENS_ICM42686_GYRO_250DPS;   break;
        case GYRO_500DPS:   cfg = 0x40; g_gyro_sens = SENS_ICM42686_GYRO_500DPS;   break;
        case GYRO_1000DPS:  cfg = 0x20; g_gyro_sens = SENS_ICM42686_GYRO_1000DPS;  break;
        case GYRO_2000DPS:  cfg = 0x00; g_gyro_sens = SENS_ICM42686_GYRO_2000DPS;  break;
        default: break;
    }
    switch (odr) {
        case GYRO_ODR_12_5HZ:  cfg |= 0x0B; break; case GYRO_ODR_25HZ:    cfg |= 0x0A; break;
        case GYRO_ODR_50HZ:    cfg |= 0x09; break; case GYRO_ODR_100HZ:   cfg |= 0x08; break;
        case GYRO_ODR_200HZ:   cfg |= 0x07; break; case GYRO_ODR_500HZ:   cfg |= 0x0F; break;
        case GYRO_ODR_1000HZ:  cfg |= 0x06; break; case GYRO_ODR_2000HZ:  cfg |= 0x05; break;
        case GYRO_ODR_4000HZ:  cfg |= 0x04; break; case GYRO_ODR_8000HZ:  cfg |= 0x03; break;
        case GYRO_ODR_16000HZ: cfg |= 0x02; break; case GYRO_ODR_32000HZ: cfg |= 0x01; break;
        default: break;
    }
    icm_write_reg(0x4F, cfg);
}

static void _icm_set_accel(ICM42686_AccFSR fsr, ICM42686_AccODR odr)
{
    uint8_t cfg = 0x00;
    switch (fsr) {
        case ACC_2G:  cfg = 0x60; g_acc_sens = SENS_ICM42686_ACC_2G;  break;
        case ACC_4G:  cfg = 0x40; g_acc_sens = SENS_ICM42686_ACC_4G;  break;
        case ACC_8G:  cfg = 0x20; g_acc_sens = SENS_ICM42686_ACC_8G;  break;
        case ACC_16G: cfg = 0x00; g_acc_sens = SENS_ICM42686_ACC_16G; break;
        default: break;
    }
    switch (odr) {
        case ACC_ODR_12_5HZ:  cfg |= 0x0B; break; case ACC_ODR_25HZ:    cfg |= 0x0A; break;
        case ACC_ODR_50HZ:    cfg |= 0x09; break; case ACC_ODR_100HZ:   cfg |= 0x08; break;
        case ACC_ODR_200HZ:   cfg |= 0x07; break; case ACC_ODR_500HZ:   cfg |= 0x0F; break;
        case ACC_ODR_1000HZ:  cfg |= 0x06; break; case ACC_ODR_2000HZ:  cfg |= 0x05; break;
        case ACC_ODR_4000HZ:  cfg |= 0x04; break; case ACC_ODR_8000HZ:  cfg |= 0x03; break;
        case ACC_ODR_16000HZ: cfg |= 0x02; break; case ACC_ODR_32000HZ: cfg |= 0x01; break;
        default: break;
    }
    icm_write_reg(0x50, cfg);
}

static void _icm_set_filters(ICM42686_BandwidthFactor gyro_bw,
                              ICM42686_FilterOrder    gyro_order,
                              ICM42686_BandwidthFactor acc_bw,
                              ICM42686_FilterOrder    acc_order)
{
    uint8_t gac = 0x00, gc1 = 0x00, ac1 = 0x00;

    switch (gyro_bw) {
        case BW_FACTOR_2: gac=0x00; break; case BW_FACTOR_4: gac=0x01; break;
        case BW_FACTOR_5: gac=0x02; break; case BW_FACTOR_8: gac=0x03; break;
        case BW_FACTOR_10: gac=0x04; break; case BW_FACTOR_16: gac=0x05; break;
        case BW_FACTOR_20: gac=0x06; break; case BW_FACTOR_40: gac=0x07; break;
        case BW_LOW_LATENCY_1: gac=0x0E; break; case BW_LOW_LATENCY_2: gac=0x0F; break;
        default: break;
    }
    switch (acc_bw) {
        case BW_FACTOR_2: gac|=0x00; break; case BW_FACTOR_4: gac|=0x10; break;
        case BW_FACTOR_5: gac|=0x20; break; case BW_FACTOR_8: gac|=0x30; break;
        case BW_FACTOR_10: gac|=0x40; break; case BW_FACTOR_16: gac|=0x50; break;
        case BW_FACTOR_20: gac|=0x60; break; case BW_FACTOR_40: gac|=0x70; break;
        case BW_LOW_LATENCY_1: gac|=0xE0; break; case BW_LOW_LATENCY_2: gac|=0xF0; break;
        default: break;
    }
    icm_write_reg(0x52, gac);

    switch (gyro_order) {
        case FILTER_1ST: gc1=0x02; break; case FILTER_2ST: gc1=0x06; break;
        case FILTER_3ST: gc1=0xA0; break; default: break;
    }
    icm_write_reg(0x51, gc1);

    switch (acc_order) {
        case FILTER_1ST: ac1=0x02; break; case FILTER_2ST: ac1=0x06; break;
        case FILTER_3ST: ac1=0xA0; break; default: break;
    }
    icm_write_reg(0x53, ac1);
}

/* ======================================================================
 *   数据读取
 * ====================================================================== */

void ICM42686_ReadData(void)
{
    uint8_t hi, lo;

    if (!g_icm42686_ready) return;

    /* 逐寄存器读，避开 burst read 兼容性问题 */
    hi = icm_read_reg(0x1F); lo = icm_read_reg(0x20);
    g_icm42686_raw.acc_x = (int16_t)(((uint16_t)hi << 8) | lo);

    hi = icm_read_reg(0x21); lo = icm_read_reg(0x22);
    g_icm42686_raw.acc_y = (int16_t)(((uint16_t)hi << 8) | lo);

    hi = icm_read_reg(0x23); lo = icm_read_reg(0x24);
    g_icm42686_raw.acc_z = (int16_t)(((uint16_t)hi << 8) | lo);

    hi = icm_read_reg(0x25); lo = icm_read_reg(0x26);
    g_icm42686_raw.gyro_x = (int16_t)(((uint16_t)hi << 8) | lo);

    hi = icm_read_reg(0x27); lo = icm_read_reg(0x28);
    g_icm42686_raw.gyro_y = (int16_t)(((uint16_t)hi << 8) | lo);

    hi = icm_read_reg(0x29); lo = icm_read_reg(0x2A);
    g_icm42686_raw.gyro_z = (int16_t)(((uint16_t)hi << 8) | lo);

    g_icm42686.gyro_x = g_icm42686_raw.gyro_x / g_gyro_sens;
    g_icm42686.gyro_y = g_icm42686_raw.gyro_y / g_gyro_sens;
    g_icm42686.gyro_z = g_icm42686_raw.gyro_z / g_gyro_sens;
    g_icm42686.acc_x  = g_icm42686_raw.acc_x  / g_acc_sens;
    g_icm42686.acc_y  = g_icm42686_raw.acc_y  / g_acc_sens;
    g_icm42686.acc_z  = g_icm42686_raw.acc_z  / g_acc_sens;
}

float ICM42686_GetAccelY(void) { return g_icm42686.acc_y; }

/* ======================================================================
 *   初始化
 * ====================================================================== */

HAL_StatusTypeDef ICM42686_Init(const ICM42686_Config *config)
{
    ICM42686_Config default_cfg = {
        GYRO_250DPS, GYRO_ODR_1000HZ,
        ACC_2G, ACC_ODR_1000HZ,
        FILTER_2ST, BW_FACTOR_4,
        FILTER_2ST, BW_FACTOR_4,
    };

    if (config == NULL) config = &default_cfg;

    /* PB4/PB5 已在 gpio.c 初始化为推挽输出 (与 OLED 一致), 无需改动 */
    SDA_H(); SCL_H();
    HAL_Delay(1);

    /* 先做芯片识别 (会自动检测地址并设置 g_icm_addr) */
    if (_find_icm() == 0xFF) {
        App_Uart6Printf("[ICM42686] FAILED: no device\r\n");
        return HAL_ERROR;
    }

    /* 软件复位 */
    icm_write_reg(0x11, 0x01);
    HAL_Delay(10);

    /* 先使能传感器 LN 模式 (参照科宇: 使能后再配置) */
    icm_write_reg(0x4E, 0x0F);
    HAL_Delay(50);

    /* 再配置各模块 */
    _icm_set_gyro(config->gyro_fsr, config->gyro_odr);
    _icm_set_accel(config->acc_fsr, config->acc_odr);
    _icm_set_filters(config->gyro_bw, config->gyro_filter_order,
                      config->acc_bw,  config->acc_filter_order);
    HAL_Delay(10);

    g_icm42686_ready = true;
    App_Uart6Printf("[ICM42686] OK!\r\n");
    return HAL_OK;
}
