#ifndef __PCA9685_H
#define __PCA9685_H

#include <stdint.h>

#define PCA9685_MIN_PULSE_US     500U
#define PCA9685_MAX_PULSE_US     2500U
#define PCA9685_PERIOD_US        20000U

static inline uint16_t PCA9685_PulseUsToTicks(uint16_t pulse_us)
{
    if (pulse_us < PCA9685_MIN_PULSE_US)
    {
        pulse_us = PCA9685_MIN_PULSE_US;
    }
    if (pulse_us > PCA9685_MAX_PULSE_US)
    {
        pulse_us = PCA9685_MAX_PULSE_US;
    }

    return (uint16_t)(((uint32_t)pulse_us * 4096U + (PCA9685_PERIOD_US / 2U)) /
                      PCA9685_PERIOD_US);
}

int32_t PCA9685_Init(void);
int32_t PCA9685_Set270Angle(float angle_deg);
int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg);

#endif
