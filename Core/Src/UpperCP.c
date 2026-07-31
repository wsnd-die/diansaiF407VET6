#include "UpperCP.h"
#include "usart.h"
#include "oled.h"
#include "cmsis_os.h"

#include <stdarg.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UPPERCP_RX_BUF_LEN 128U
#define UPPERCP_CMD_MOVE_GBK "\xD2\xC6\xB6\xAF"

static volatile uint8_t uppercp_rx_buf[UPPERCP_RX_BUF_LEN];
static volatile uint16_t uppercp_rx_head;
static volatile uint16_t uppercp_rx_tail;
static volatile uint32_t uppercp_rx_count;
static volatile uint8_t uppercp_last_byte;
static char uppercp_cmd_buf[UPPERCP_RX_BUF_LEN];
static char uppercp_last_cmd[UPPERCP_RX_BUF_LEN];
static uint16_t uppercp_cmd_len;
static uint8_t uppercp_receiving;
int16_t gangzhu_speed = 0;

static void Serial5_Printf(const char *fmt, ...)
{
    char tx_buf[96];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    if ((uint32_t)len >= sizeof(tx_buf)) {
        len = (int)sizeof(tx_buf) - 1;
    }

    /* 等待上一次 DMA 发送完成 */
    while (huart5.gState == HAL_UART_STATE_BUSY_TX) {
        osDelay(1);
    }
    (void)HAL_UART_Transmit_DMA(&huart5, (uint8_t *)tx_buf, (uint16_t)len);
}

void UpperCP_UartRxByte(uint8_t data)
{
    uint16_t next_head = (uint16_t)((uppercp_rx_head + 1U) % UPPERCP_RX_BUF_LEN);

    uppercp_last_byte = data;
    uppercp_rx_count++;

    if (next_head == uppercp_rx_tail) {
        return;
    }

    uppercp_rx_buf[uppercp_rx_head] = data;
    uppercp_rx_head = next_head;
}

void UpperCP_SendTask(const char *task)
{
    static const char line_end[] = "\r\n";

    if (task == NULL) {
        return;
    }

    if ((strcmp(task, "send") != 0) &&
        (strcmp(task, "pour") != 0) &&
        (strcmp(task, "scan") != 0)) {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)task, (uint16_t)strlen(task), 100U);
    (void)HAL_UART_Transmit(&huart5, (uint8_t *)line_end, (uint16_t)(sizeof(line_end) - 1U), 100U);
}

const char *UpperCP_GetLastCommand(void)
{
    return uppercp_last_cmd;
}

uint32_t UpperCP_GetRxCount(void)
{
    return uppercp_rx_count;
}

uint8_t UpperCP_GetLastByte(void)
{
    return uppercp_last_byte;
}

static char *ret = NULL;
uint8_t PosFlag = 1;
int16_t gangzhu_err = 0;
volatile uint8_t gangzhu_err_updated = 0U;
uint8_t CameraFlag = 0;

/* uint8_t fruits[8] = {3,5,7,1,6,10,12,9}; */
uint8_t fruits[8] = {4,3,1,10,8,9,2,11};
/* uint8_t fruits[8] = {12,2,9,4,5,11,1,7}; */

uint8_t fruits_count = 0;
/* 包头为 %，包尾为 #，数据格式为“距离速度”，例如 %+25-35#。 */
void UpperCP_RX(void)
{
    uint8_t command_ready = 0U;

    while (uppercp_rx_tail != uppercp_rx_head) {
        uint8_t ch = uppercp_rx_buf[uppercp_rx_tail];
        uppercp_rx_tail = (uint16_t)((uppercp_rx_tail + 1U) % UPPERCP_RX_BUF_LEN);

        if (ch == '%') {
            uppercp_cmd_len = 0U;
            uppercp_receiving = 1U;
            continue;
        }

        if (uppercp_receiving == 0U) {
            continue;
        }

        if (ch == '#') {
            if (uppercp_cmd_len != 0U) {
                uppercp_cmd_buf[uppercp_cmd_len] = '\0';
                command_ready = 1U;
            }
            uppercp_cmd_len = 0U;
            uppercp_receiving = 0U;

            if (command_ready != 0U) {
                break;
            }

            continue;
        }

        if (uppercp_cmd_len < (UPPERCP_RX_BUF_LEN - 1U)) {
            uppercp_cmd_buf[uppercp_cmd_len++] = (char)ch;
        } else {
            uppercp_cmd_len = 0U;
            uppercp_receiving = 0U;
        }
    }

    if (command_ready == 0U) {
        return;
    }

    {
        char *distance_end;
        char *speed_end;
        long distance = strtol(uppercp_cmd_buf, &distance_end, 10);
        long speed = strtol(distance_end, &speed_end, 10);

        if ((distance_end == uppercp_cmd_buf) ||
            (speed_end == distance_end) ||
            (*speed_end != '\0') ||
            (distance < INT16_MIN) || (distance > INT16_MAX) ||
            (speed < INT16_MIN) || (speed > INT16_MAX)) {
            return;
        }
        else{
        gangzhu_err = (int16_t)distance;
        gangzhu_speed = (int16_t)speed;
        gangzhu_err_updated=1;
        }
    }
}

void cmd_func(void)
{
    if (memcmp(ret, "cmd", 3) == 0) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            Serial5_Printf("num = %f\r\n", temp_num);
        }

        *ret = 0;
    }
}

void speed_func(void)
{
    if (memcmp(ret, "speed", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        /* speed struct removed with navigation.c */
        (void)temp_num;
        *ret = 0;
    }
}

void angle_func(void)
{
    if (memcmp(ret, "angle", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        /* angle_speed struct removed with navigation.c */
        (void)temp_num;
        *ret = 0;
    }
}

void face_func(void)
{
    if (memcmp(ret, "face", 4) == 0) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
        }

        /* TarAngle removed with navigation.c */
        (void)temp_num;
        *ret = 0;
    }
}

void voice_func(void)
{
    if (memcmp(ret, "voice", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        /* Voice_Num(temp_num); -- voice.c removed */
        (void)temp_num;

        *ret = 0;
    }
}

void Arm_func(void)
{
    if (memcmp(ret, "arm", 3) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        (void)temp_num;
        *ret = 0;
    }
}

void ErWeiMa_func(void)
{
    if (memcmp(ret, "QR", 2) == 0) {
        float temp_num = 0.0f;
        char *p_num;
        uint8_t i = 0U;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            if (i < 8U) {
                fruits[i++] = (uint8_t)temp_num;
            }
        }

        fruits_count = i;
        CameraFlag = 1U;

        *ret = 0;
    }
}

void Move_func(void)
{
    if ((memcmp(ret, UPPERCP_CMD_MOVE_GBK, 4) == 0) || (memcmp(ret, "move", 4) == 0)) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            Serial5_Printf("move %.2f\r\n", temp_num);
        }

        /* TarPos removed with navigation.c */
        (void)temp_num;
        *ret = 0;
    }
}
