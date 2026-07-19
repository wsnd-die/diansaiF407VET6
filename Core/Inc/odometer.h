#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "main.h"

/* 外部可访问的里程计数据接收缓存 */
extern uint8_t odom_rx_buf[];
/* 外部可访问的当前左右轮平均单步位移量（单位：mm） */
extern float motor_half_delta_mm;

/**
 * @brief 初始化里程计，清零状态变量，配置串口中断
 */
void Odometer_Init(void);

/**
 * @brief 里程计更新轮询函数。应定期（如50ms）在主循环或高优先级任务中调用。
 *        函数中交替向左前和右前轮驱动器发送读取位置参数指令。
 */
void Odometer_Update(void);

/**
 * @brief 串口接收中断回调字节处理函数，供 UART 中断服务程序调用
 * @param data 串口接收到的单字节数据
 */
void Odometer_UartRxByte(uint8_t data);

#endif

