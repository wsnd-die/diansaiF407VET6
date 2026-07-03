#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

void Servo_Init(void);

/* PB1: TIM3_CH4, 270度舵机，0度为中位，范围 -135 ~ +135 度。 */
void Servo_SetPulseUs(uint16_t pulse_us);
void Servo_SetAngle(float angle_deg);

/* PC6: TIM8_CH1, 180度舵机，0度为中位，范围 -90 ~ +90 度。 */
void Servo180_SetPulseUs(uint16_t pulse_us);
void Servo180_SetAngle(float angle_deg);

#endif
