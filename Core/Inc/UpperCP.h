#ifndef UPPERCP_H
#define UPPERCP_H

#include <stdint.h>

extern uint8_t PosFlag;
extern int16_t gangzhu_err;
extern volatile uint8_t gangzhu_err_updated;
extern uint8_t fruits[8];
extern uint8_t fruits_count;
extern int16_t gangzhu_distance;
extern int16_t gangzhu_speed;
extern uint8_t CameraFlag;

void UpperCP_RX(void);
void UpperCP_UartRxByte(uint8_t data);
void UpperCP_SendTask(const char *task);
const char *UpperCP_GetLastCommand(void);
uint32_t UpperCP_GetRxCount(void);
uint8_t UpperCP_GetLastByte(void);

void cmd_func(void);
void speed_func(void);
void angle_func(void);
void face_func(void);
void voice_func(void);
void Arm_func(void);
void ErWeiMa_func(void);
void Move_func(void);

#endif

