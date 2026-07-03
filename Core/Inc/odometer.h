#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "main.h"

extern volatile float g_odometer_distance_cm;
extern volatile float g_odometer_delta_cm;
extern volatile float g_odometer_motor_angle_deg;

void Odometer_Init(void);
void Odometer_Update(void);
void Odometer_UartRxByte(uint8_t data);

float Odometer_GetDistanceCm(void);
float Odometer_GetMotorAngleDeg(void);

#endif
