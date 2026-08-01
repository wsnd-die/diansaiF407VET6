/**
 * @file    imu660rc.c
 * @brief   IMU660RC (ICM-42688-P) 精简驱动 — 软件 I2C, 仅 Y 轴加速度
 * @note    PB4=SDA, PB5=SCL, I2C 地址 0x6B
 */

#include "imu660rc.h"
#include "app.h"

/* ========== GPIO 操作 ========== */
#define SCL_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define SDA_RD()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)

static void sda_input(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_4;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void sda_output(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_4;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* I2C 延时 (~2us @168MHz) */
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

static uint8_t i2c_wait_ack(void)
{
    uint8_t ack;
    sda_input();
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
    if (ack) SDA_L(); else SDA_H();
    i2c_delay();
    SCL_H(); i2c_delay();
    SCL_L(); i2c_delay();
    SDA_H();
    return data;
}

/* ========== 寄存器操作 ========== */

static uint8_t g_dev_addr = IMU660RC_I2C_ADDR;

static void imu_write_reg(uint8_t reg, uint8_t data)
{
    i2c_start();
    i2c_write_byte((uint8_t)(g_dev_addr << 1));
    i2c_write_byte(reg);
    i2c_write_byte(data);
    i2c_stop();
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t val;
    i2c_start();
    i2c_write_byte((uint8_t)(g_dev_addr << 1));
    i2c_write_byte(reg);
    i2c_start();
    i2c_write_byte((uint8_t)((g_dev_addr << 1) | 1U));
    val = i2c_read_byte(0);
    i2c_stop();
    return val;
}

/* ========== 自检 ========== */

static bool imu_self_check(void)
{
    uint8_t id;
    uint8_t found = 0;

    /* 扫描 0x6A / 0x6B */
    App_Uart6Printf("[IMU660RC] scanning I2C...\r\n");
    for (uint8_t a = IMU660RC_I2C_ADDR_ALT; a <= IMU660RC_I2C_ADDR; a++) {
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
        uint8_t ack = SDA_RD() ? 1U : 0U;
        SCL_L(); i2c_delay();
        i2c_stop();

        App_Uart6Printf("      0x%02X %s\r\n", a, ack == 0 ? "ACK" : "NACK");
        if (ack == 0) found = a;
    }

    if (found == 0) {
        App_Uart6Printf("[IMU660RC] no device!\r\n");
        return false;
    }

    g_dev_addr = found;
    App_Uart6Printf("[IMU660RC] addr=0x%02X\r\n", g_dev_addr);

    /* 读 CHIP_ID */
    id = imu_read_reg(IMU660RC_CHIP_ID);
    App_Uart6Printf("[IMU660RC] CHIP_ID=0x%02X\r\n", id);
    if (id != IMU660RC_CHIP_ID_VAL) {
        App_Uart6Printf("[IMU660RC] bad ID (expected 0x%02X)\r\n", IMU660RC_CHIP_ID_VAL);
        return false;
    }
    return true;
}

/* ========== 初始化 ========== */

bool imu660rc_init(void)
{
    SDA_H(); SCL_H();
    HAL_Delay(1);

    if (!imu_self_check()) {
        return false;
    }

    /* 复位 */
    imu_write_reg(IMU660RC_FUNC_CFG_ACCESS, 0x04);
    HAL_Delay(30);

    /* 接口配置 */
    imu_write_reg(IMU660RC_CTRL3, 0x44);

    /* 加速度计: ±2g, 低噪声模式 */
    imu_write_reg(IMU660RC_CTRL8, 0x00);

    /* CTRL1: 低噪声模式, 1kHz ODR */
    imu_write_reg(IMU660RC_CTRL1, 0x15);

    App_Uart6Printf("[IMU660RC] OK!\r\n");
    return true;
}

static float g_acc_offset  = 0.0f;    /* 零偏校准值 */
static float g_acc_filtered = 0.0f;    /* 低通滤波后的值 */
static bool  g_calibrated   = false;

/* ========== 读 Y 轴加速度 ========== */

float imu660rc_read_acc_y(void)
{
    uint8_t lo, hi;
    int16_t raw;

    hi = imu_read_reg(IMU660RC_OUTY_H_A);
    lo = imu_read_reg(IMU660RC_OUTY_L_A);
    raw = (int16_t)(((uint16_t)hi << 8) | lo);

    return (float)raw / IMU660RC_ACC_SENS_2G;
}

/* ========== 滤波后的 Y 轴加速度 ========== */

float imu660rc_get_acc_y_filtered(void)
{
    float raw = imu660rc_read_acc_y() - g_acc_offset;

    /* 死区：微小抖动归零，避免噪声被后续增益放大 */
    if (raw < IMU660RC_DEADBAND && raw > -IMU660RC_DEADBAND) {
        raw = 0.0f;
    }

    /* 一阶低通: filtered = alpha * filtered + (1-alpha) * raw */
    g_acc_filtered = IMU660RC_FILTER_ALPHA * g_acc_filtered
                   + (1.0f - IMU660RC_FILTER_ALPHA) * raw;

    return g_acc_filtered;
}

/* ========== 零偏校准 ========== */

void imu660rc_calibrate(void)
{
    float sum = 0.0f;

    App_Uart6Printf("[IMU660RC] calibrating (%d samples)...\r\n", IMU660RC_CALIB_SAMPLES);

    for (int i = 0; i < IMU660RC_CALIB_SAMPLES; i++) {
        sum += imu660rc_read_acc_y();
        HAL_Delay(5);  /* 5ms 间隔, 合计约 1s */
    }

    g_acc_offset = sum / (float)IMU660RC_CALIB_SAMPLES;
    g_calibrated = true;
    g_acc_filtered = 0.0f;

    App_Uart6Printf("[IMU660RC] calib done, offset=%.4f g\r\n", g_acc_offset);
}

bool imu660rc_is_calibrated(void)
{
    return g_calibrated;
}
