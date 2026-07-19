#ifndef __HWT101_HAL_H
#define __HWT101_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hwt101.h"
#include "stm32f4xx_hal.h"

/* ---- 解析后的角度（由 HWT101_ParsePacket 更新） ---- */
extern volatile float g_hwt101_roll;
extern volatile float g_hwt101_pitch;
extern volatile float g_hwt101_yaw;
extern volatile uint8_t g_hwt101_data_ready;

/**
 * @brief  初始化 HWT101 陀螺仪（Normal 协议，UART4，115200）
 * @note   调用前需确保 HAL 已初始化、UART4 已配置
 *         RX 数据由 UART4_IRQHandler 直接解析，不经过 HAL RX 回调
 * @retval 0=成功，其他=错误码
 */
int32_t HWT101_HAL_Init(void);

/**
 * @brief  解析一帧 11 字节数据包（由 UART4_IRQHandler 调用）
 * @param  data: 指向 11 字节缓冲区
 * @note   校验成功后更新 g_hwt101_roll/pitch/yaw
 *         0x53 包 → 角度（归一化 int16, ±32768 → ±180°）
 */
void HWT101_ParsePacket(uint8_t *data);


/**
 * @brief  读取相对初始化零点的 Yaw 角（度，-180°~180°）
 */
float Get_zeroYaw(void);

/**
 * @brief  读取指定寄存器的原始 int16 值
 * @param  reg: 寄存器地址（参考 REG.h 中的宏定义）
 * @retval 寄存器值（int16_t）
 */
int16_t HWT101_ReadReg(uint32_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __HWT101_HAL_H */
