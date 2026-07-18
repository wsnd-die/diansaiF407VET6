#include "pca9685.h"
#include "i2c.h"

#define PCA9685_ADDRESS          (0x40U << 1)
#define PCA9685_MODE1            0x00U
#define PCA9685_PRESCALE         0xFEU
#define PCA9685_LED0_ON_L        0x06U
#define PCA9685_PRESCALE_50HZ    121U
#define PCA9685_270_CENTER_DEG   10.0f

static int32_t PCA9685_Write(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS, reg,
                          I2C_MEMADD_SIZE_8BIT, data, len,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

static int32_t PCA9685_SetTicks(uint8_t channel, uint16_t off_ticks)
{
    uint8_t data[4];

    if (channel > 15U)
    {
        return -1;
    }

    data[0] = 0U;
    data[1] = 0U;
    data[2] = (uint8_t)(off_ticks & 0xFFU);
    data[3] = (uint8_t)(off_ticks >> 8);

    return PCA9685_Write((uint8_t)(PCA9685_LED0_ON_L + 4U * channel), data, sizeof(data));
}

static uint16_t PCA9685_AngleToPulseUs(float angle_deg, float max_angle_deg)
{
    float pulse_us;

    if (angle_deg < -max_angle_deg)
    {
        angle_deg = -max_angle_deg;
    }
    if (angle_deg > max_angle_deg)
    {
        angle_deg = max_angle_deg;
    }

    pulse_us = 1500.0f + angle_deg * 1000.0f / max_angle_deg;
    return (uint16_t)(pulse_us + 0.5f);
}

int32_t PCA9685_Init(void)
{
    uint8_t value;

    value = 0x10U;
    if (PCA9685_Write(PCA9685_MODE1, &value, 1U) != 0)
    {
        return -1;
    }

    value = PCA9685_PRESCALE_50HZ;
    if (PCA9685_Write(PCA9685_PRESCALE, &value, 1U) != 0)
    {
        return -1;
    }

    value = 0x00U;
    if (PCA9685_Write(PCA9685_MODE1, &value, 1U) != 0)
    {
        return -1;
    }

    HAL_Delay(1U);
    value = 0xA1U;
    return PCA9685_Write(PCA9685_MODE1, &value, 1U);
}

int32_t PCA9685_Set270Angle(float angle_deg)
{
    return PCA9685_SetTicks(0U, PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg + PCA9685_270_CENTER_DEG, 135.0f)));
}

int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg)
{
    if (channel < 1U || channel > 4U)
    {
        return -1;
    }

    return PCA9685_SetTicks(channel, PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg, 90.0f)));
}
