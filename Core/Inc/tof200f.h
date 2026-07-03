#ifndef __TOF200F_H
#define __TOF200F_H

#include "main.h"

/* 旧工程变量名：TOF200F 原始距离值；旧工程用 TofData / 10.0 得到 cm。 */
extern volatile float TofData;

void TOF200F_Init(void);
void TOF200F_UartRxByte(uint8_t data);
float TOF200F_GetDistanceCm(void);
void get_dis(void);

#endif
