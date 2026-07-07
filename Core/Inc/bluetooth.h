#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Bluetooth_Init(void);
HAL_StatusTypeDef Bluetooth_SendData(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef Bluetooth_SendString(const char *str);
void Bluetooth_UartRxByte(uint8_t data);
uint8_t Bluetooth_ReadByte(uint8_t *data);
uint16_t Bluetooth_ReadLine(char *buf, uint16_t buf_len);
uint8_t Bluetooth_IsLineReady(void);
uint8_t Bluetooth_IsOverflow(void);
void Bluetooth_ClearOverflow(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_H */
