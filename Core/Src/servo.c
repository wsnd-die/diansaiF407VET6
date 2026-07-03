#include "servo.h"

#define SERVO_MIN_PULSE_US      500U
#define SERVO_CENTER_PULSE_US   1500U
#define SERVO_MAX_PULSE_US      2500U
#define SERVO270_MAX_ANGLE_DEG  135.0f
#define SERVO180_MAX_ANGLE_DEG  90.0f

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

static uint16_t Servo_ClampPulseUs(uint16_t pulse_us)
{
  if (pulse_us < SERVO_MIN_PULSE_US)
  {
    return SERVO_MIN_PULSE_US;
  }
  if (pulse_us > SERVO_MAX_PULSE_US)
  {
    return SERVO_MAX_PULSE_US;
  }
  return pulse_us;
}

/* PB1: TIM3_CH4, 270度舵机，直接设置高电平脉宽，单位 us。 */
void Servo_SetPulseUs(uint16_t pulse_us)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Servo_ClampPulseUs(pulse_us));
}

/* PB1: 0度为中位，正角度向正方向转，范围限制在 -135 ~ +135 度。 */
void Servo_SetAngle(float angle_deg)
{
  float pulse_us;

  if (angle_deg < -SERVO270_MAX_ANGLE_DEG)
  {
    angle_deg = -SERVO270_MAX_ANGLE_DEG;
  }
  else if (angle_deg > SERVO270_MAX_ANGLE_DEG)
  {
    angle_deg = SERVO270_MAX_ANGLE_DEG;
  }

  pulse_us = SERVO_CENTER_PULSE_US +
             angle_deg * (float)(SERVO_MAX_PULSE_US - SERVO_CENTER_PULSE_US) / SERVO270_MAX_ANGLE_DEG;
  Servo_SetPulseUs((uint16_t)(pulse_us + 0.5f));
}

/* PC6: TIM8_CH1, 180度舵机，直接设置高电平脉宽，单位 us。 */
void Servo180_SetPulseUs(uint16_t pulse_us)
{
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Servo_ClampPulseUs(pulse_us));
}

/* PC6: 0度为中位，正角度向正方向转，范围限制在 -90 ~ +90 度。 */
void Servo180_SetAngle(float angle_deg)
{
  float pulse_us;

  if (angle_deg < -SERVO180_MAX_ANGLE_DEG)
  {
    angle_deg = -SERVO180_MAX_ANGLE_DEG;
  }
  else if (angle_deg > SERVO180_MAX_ANGLE_DEG)
  {
    angle_deg = SERVO180_MAX_ANGLE_DEG;
  }

  pulse_us = SERVO_CENTER_PULSE_US +
             angle_deg * (float)(SERVO_MAX_PULSE_US - SERVO_CENTER_PULSE_US) / SERVO180_MAX_ANGLE_DEG;
  Servo180_SetPulseUs((uint16_t)(pulse_us + 0.5f));
}

void Servo_Init(void)
{
  Servo_SetPulseUs(SERVO_CENTER_PULSE_US);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  Servo180_SetPulseUs(SERVO_CENTER_PULSE_US);
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}
