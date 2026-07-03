#include "tof200f.h"

extern UART_HandleTypeDef huart1;

/*
 * TOF200F 单次测距命令，来自旧工程：
 * 01 03 00 10 00 01 85 CF
 */
static uint8_t tof200f_start_single[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};

/* 回包格式：01 03 02 距离高字节 距离低字节 CRC低 CRC高。 */
#define TOF200F_RX_FRAME_LEN 7

volatile float TofData = 0.0f;

static uint8_t tof200f_rx_buf[TOF200F_RX_FRAME_LEN];
static uint8_t tof200f_rx_index = 0;

void TOF200F_Init(void)
{
    tof200f_rx_index = 0;
    TofData = 0.0f;

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void get_dis(void)
{
    (void)HAL_UART_Transmit(&huart1, tof200f_start_single, sizeof(tof200f_start_single), HAL_MAX_DELAY);
}

void TOF200F_UartRxByte(uint8_t data)
{
    if (tof200f_rx_index == 0 && data != 0x01)
    {
        return;
    }

    if (tof200f_rx_index == 1 && data != 0x03)
    {
        tof200f_rx_index = 0;
        return;
    }

    if (tof200f_rx_index == 2 && data != 0x02)
    {
        tof200f_rx_index = 0;
        return;
    }

    tof200f_rx_buf[tof200f_rx_index++] = data;

    if (tof200f_rx_index >= TOF200F_RX_FRAME_LEN)
    {
        TofData = (float)(((uint16_t)tof200f_rx_buf[3] << 8) | tof200f_rx_buf[4]);
        tof200f_rx_index = 0;
    }
}

float TOF200F_GetDistanceCm(void)
{
    return TofData / 10.0f;
}
