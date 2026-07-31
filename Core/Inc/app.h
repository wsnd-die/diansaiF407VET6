#ifndef __APP_H
#define __APP_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化应用层调试状态
 */
void App_Init(void);

/**
 * @brief 串口指令接收处理函数（中断上下文，仅存标志位）
 * @param data 接收到的串口数据字节
 */
void App_CommandUartRxByte(uint8_t data);

/**
 * @brief 处理串口调试命令：电机控制 / PID 参数调节
 * @note  必须在任务上下文中调用，不可在中断里调用
 */
void App_ProcessCommand(void);

/**
 * @brief USART6 格式化输出，调试用蓝牙串口发送
 * @param fmt  printf 格式字符串
 * @param ...  可变参数
 */
void App_Uart6Printf(const char *fmt, ...);

/**
 * @brief PID 日志开关，调试用
 * @return true 开启，false 关闭
 */
bool App_IsPidLogEnabled(void);
uint32_t App_GetUart6RxCount(void);
uint8_t App_GetUart6LastByte(void);

#endif
