#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "bujin.h"
#include "gangzhu_pid.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_CMD_LINE_LEN 96U

static volatile uint8_t s_data;
static volatile uint8_t s_line_ready;
static volatile uint8_t s_line_len;
static volatile uint32_t s_uart6_rx_count;
static volatile uint8_t s_uart6_last_byte;
static char s_line_buf[APP_CMD_LINE_LEN];
static float s_pid_step = 0.1f;
static bool s_pid_log_enabled = true;

void App_Uart6Printf(const char *fmt, ...)
{
    char tx_buf[384];
    va_list args;
    int tx_len;

    va_start(args, fmt);
    tx_len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    if (tx_len <= 0) {
        return;
    }

    if ((uint32_t)tx_len >= sizeof(tx_buf)) {
        tx_len = (int)sizeof(tx_buf) - 1;
    }

    (void)HAL_UART_Transmit(&huart6, (uint8_t *)tx_buf, (uint16_t)tx_len, 100U);
}

void App_CommandUartRxByte(uint8_t data)
{
    s_uart6_rx_count++;
    s_uart6_last_byte = data;

    if ((data == '\r') || (data == '\n')) {
        if ((s_line_len > 0U) && (s_line_ready == 0U)) {
            s_line_buf[s_line_len] = '\0';
            s_line_ready = 1U;
        }
        s_line_len = 0U;
        return;
    }

    if ((data == '@') || (s_line_len > 0U)) {
        if (s_line_ready != 0U) {
            return;
        }

        if (s_line_len < (APP_CMD_LINE_LEN - 1U)) {
            s_line_buf[s_line_len++] = (char)data;
        } else {
            s_line_len = 0U;
        }
        return;
    }

    s_data = data;
}

static void App_PrintPid(void)
{
    App_Uart6Printf("pidack,gain,kp=%.4f,ki=%.4f,kd=%.4f\r\n",
                    s_gangzhu_pid.kp,
                    s_gangzhu_pid.ki,
                    s_gangzhu_pid.kd);
}

static void App_PrintSpeedPid(void)
{
    App_Uart6Printf("pidack,vgain,kp=%.4f,ki=%.4f,kd=%.4f,target=%.2f,outer=%d,ven=%d\r\n",
                    s_gangzhu_pid.speed_pid.Kp,
                    s_gangzhu_pid.speed_pid.Ki,
                    s_gangzhu_pid.speed_pid.Kd,
                    s_gangzhu_pid.target_speed,
                    s_gangzhu_pid.outer_enabled ? 1 : 0,
                    s_gangzhu_pid.speed_enabled ? 1 : 0);
}

static void App_ProcessLineCommand(const char *line)
{
    float kp;
    float ki;
    float kd;
    float value;
    char arg[16];

    if (strcmp(line, "@pid?") == 0) {
        App_PrintPid();
    } else if (sscanf(line, "@pid %f %f %f", &kp, &ki, &kd) == 3) {
        GangzhuPid_SetGains(&s_gangzhu_pid, kp, ki, kd);
        App_PrintPid();
    } else if (strcmp(line, "@vpid?") == 0) {
        App_PrintSpeedPid();
    } else if (sscanf(line, "@vpid %f %f %f", &kp, &ki, &kd) == 3) {
        GangzhuPid_SetSpeedGains(&s_gangzhu_pid, kp, ki, kd);
        App_PrintSpeedPid();
    } else if (sscanf(line, "@vtarget %f", &value) == 1) {
        s_gangzhu_pid.target_speed = value;
        s_gangzhu_pid.speed_enabled = true;
        App_Uart6Printf("pidack,vtarget,target=%.2f\r\n", s_gangzhu_pid.target_speed);
    } else if (sscanf(line, "@outer %15s", arg) == 1) {
        if (strcmp(arg, "on") == 0) {
            GangzhuPid_SetOuterEnabled(&s_gangzhu_pid, true);
        } else if (strcmp(arg, "off") == 0) {
            GangzhuPid_SetOuterEnabled(&s_gangzhu_pid, false);
        }
        App_Uart6Printf("pidack,outer,enabled=%d\r\n", s_gangzhu_pid.outer_enabled ? 1 : 0);
    } else if (sscanf(line, "@speed %15s", arg) == 1) {
        if (strcmp(arg, "on") == 0) {
            s_gangzhu_pid.speed_enabled = true;
        } else if (strcmp(arg, "off") == 0) {
            s_gangzhu_pid.speed_enabled = false;
            GangzhuPid_ResetState(&s_gangzhu_pid);
        }
        App_Uart6Printf("pidack,speed,enabled=%d\r\n", s_gangzhu_pid.speed_enabled ? 1 : 0);
    } else if (sscanf(line, "@log %15s", arg) == 1) {
        if (strcmp(arg, "on") == 0) {
            s_pid_log_enabled = true;
        } else if (strcmp(arg, "off") == 0) {
            s_pid_log_enabled = false;
        }
        App_Uart6Printf("pidack,log,enabled=%d\r\n", s_pid_log_enabled ? 1 : 0);
    } else if (strcmp(line, "@zero") == 0) {
        GangzhuPid_ResetState(&s_gangzhu_pid);
        App_Uart6Printf("pidack,zero\r\n");
    } else if (strncmp(line, "@mark ", 6U) == 0) {
        App_Uart6Printf("pidmark,%s\r\n", &line[6]);
    } else if (strcmp(line, "@help") == 0) {
        App_Uart6Printf("pidack,help,@pid @pid? @vpid @vpid? @vtarget @outer @speed @log @zero @mark\r\n");
    } else {
        App_Uart6Printf("piderr,unknown,%s\r\n", line);
    }
}

static void App_ProcessByteCommand(uint8_t data)
{
    if (data == 'a' || data == 'A') {
        return;
    } else if (data == 's' || data == 'S') {
        Emm_V5_Pos_Control_ByPulse(5, 0, 50, 0, 100.0f, 1, false);
    } else if (data == 't' || data == 'T') {
        Emm_V5_Trigger_Zero(5, EMM_V5_ZERO_SINGLE_NEAREST, false);
    } else if (data == 'P') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, s_pid_step, 0.0f, 0.0f);
        App_PrintPid();
    } else if (data == 'p') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, -s_pid_step, 0.0f, 0.0f);
        App_PrintPid();
    } else if (data == 'I') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, s_pid_step, 0.0f);
        App_PrintPid();
    } else if (data == 'i') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, -s_pid_step, 0.0f);
        App_PrintPid();
    } else if (data == 'D') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, 0.0f, s_pid_step);
        App_PrintPid();
    } else if (data == 'd') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, 0.0f, -s_pid_step);
        App_PrintPid();
    } else if (data == 'Q') {
        if (s_pid_step >= 0.9f) {
            s_pid_step = 0.1f;
        } else if (s_pid_step >= 0.09f) {
            s_pid_step = 0.01f;
        } else {
            s_pid_step = 1.0f;
        }
        App_Uart6Printf("pidack,step,value=%.2f\r\n", s_pid_step);
    } else if (data == 'J') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, s_pid_step, 0.0f, 0.0f);
        App_PrintSpeedPid();
    } else if (data == 'j') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, -s_pid_step, 0.0f, 0.0f);
        App_PrintSpeedPid();
    } else if (data == 'K') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, s_pid_step, 0.0f);
        App_PrintSpeedPid();
    } else if (data == 'k') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, -s_pid_step, 0.0f);
        App_PrintSpeedPid();
    } else if (data == 'L') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, 0.0f, s_pid_step);
        App_PrintSpeedPid();
    } else if (data == 'l') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, 0.0f, -s_pid_step);
        App_PrintSpeedPid();
    } else if (data == 'z') {
        s_gangzhu_pid.speed_enabled = !s_gangzhu_pid.speed_enabled;
        App_PrintSpeedPid();
    } else if (data == 'W') {
        s_gangzhu_pid.target_speed += s_pid_step;
        App_Uart6Printf("pidack,vtarget,target=%.2f\r\n", s_gangzhu_pid.target_speed);
    } else if (data == 'w') {
        s_gangzhu_pid.target_speed -= s_pid_step;
        App_Uart6Printf("pidack,vtarget,target=%.2f\r\n", s_gangzhu_pid.target_speed);
    }
}

static void App_PollUart6Rx(void)
{
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET) {
        uint8_t data = (uint8_t)(huart6.Instance->DR & 0xFFU);
        App_CommandUartRxByte(data);
    }

    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart6);
    }
}

void App_ProcessCommand(void)
{
    char line[APP_CMD_LINE_LEN];
    uint8_t data;

    App_PollUart6Rx();

    if (s_line_ready != 0U) {
        (void)strncpy(line, s_line_buf, sizeof(line));
        line[sizeof(line) - 1U] = '\0';
        s_line_ready = 0U;
        App_ProcessLineCommand(line);
        return;
    }

    data = s_data;
    if (data == 0U) {
        return;
    }
    s_data = 0U;
    App_ProcessByteCommand(data);
}

bool App_IsPidLogEnabled(void)
{
    return s_pid_log_enabled;
}

uint32_t App_GetUart6RxCount(void)
{
    return s_uart6_rx_count;
}

uint8_t App_GetUart6LastByte(void)
{
    return s_uart6_last_byte;
}

void App_Init(void)
{
    s_data = 0U;
    s_line_ready = 0U;
    s_line_len = 0U;
    s_uart6_rx_count = 0U;
    s_uart6_last_byte = 0U;
    s_pid_log_enabled = true;
}
