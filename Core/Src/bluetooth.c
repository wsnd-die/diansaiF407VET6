#include "bluetooth.h"

#define BLUETOOTH_RX_BUF_SIZE 128U

extern UART_HandleTypeDef huart6;

static volatile uint8_t bluetooth_rx_buf[BLUETOOTH_RX_BUF_SIZE];
static volatile uint16_t bluetooth_rx_head = 0;
static volatile uint16_t bluetooth_rx_tail = 0;
static volatile uint8_t bluetooth_line_ready = 0;
static volatile uint8_t bluetooth_rx_overflow = 0;

static uint16_t Bluetooth_NextIndex(uint16_t index)
{
    index++;
    if (index >= BLUETOOTH_RX_BUF_SIZE)
    {
        index = 0;
    }
    return index;
}

void Bluetooth_Init(void)
{
    bluetooth_rx_head = 0;
    bluetooth_rx_tail = 0;
    bluetooth_line_ready = 0;
    bluetooth_rx_overflow = 0;

    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
}

HAL_StatusTypeDef Bluetooth_SendData(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(&huart6, (uint8_t *)data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Bluetooth_SendString(const char *str)
{
    uint16_t len = 0;

    if (str == NULL)
    {
        return HAL_ERROR;
    }

    while (str[len] != '\0')
    {
        len++;
    }

    return Bluetooth_SendData((const uint8_t *)str, len);
}

void Bluetooth_UartRxByte(uint8_t data)
{
    uint16_t next_head = Bluetooth_NextIndex(bluetooth_rx_head);

    if (next_head == bluetooth_rx_tail)
    {
        bluetooth_rx_overflow = 1;
        return;
    }

    bluetooth_rx_buf[bluetooth_rx_head] = data;
    bluetooth_rx_head = next_head;

    if (data == '\n')
    {
        bluetooth_line_ready = 1;
    }
}

uint8_t Bluetooth_ReadByte(uint8_t *data)
{
    if ((data == NULL) || (bluetooth_rx_tail == bluetooth_rx_head))
    {
        return 0;
    }

    *data = bluetooth_rx_buf[bluetooth_rx_tail];
    bluetooth_rx_tail = Bluetooth_NextIndex(bluetooth_rx_tail);
    return 1;
}

uint16_t Bluetooth_ReadLine(char *buf, uint16_t buf_len)
{
    uint16_t count = 0;
    uint8_t data;

    if ((buf == NULL) || (buf_len == 0U))
    {
        return 0;
    }

    while ((count + 1U) < buf_len)
    {
        if (Bluetooth_ReadByte(&data) == 0U)
        {
            break;
        }

        if (data == '\r')
        {
            continue;
        }

        if (data == '\n')
        {
            break;
        }

        buf[count++] = (char)data;
    }

    buf[count] = '\0';

    if (bluetooth_rx_tail == bluetooth_rx_head)
    {
        bluetooth_line_ready = 0;
    }

    return count;
}

uint8_t Bluetooth_IsLineReady(void)
{
    return bluetooth_line_ready;
}

uint8_t Bluetooth_IsOverflow(void)
{
    return bluetooth_rx_overflow;
}

void Bluetooth_ClearOverflow(void)
{
    bluetooth_rx_overflow = 0;
}
