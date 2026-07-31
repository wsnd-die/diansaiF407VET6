#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "bujin.h"
#include "gangzhu_pid.h"
#include <stdarg.h>
#include <stdio.h>

/* 串口命令接收标志位 */
static volatile uint8_t s_data;

/* PID 调试步距，默认 0.1 */
static float s_pid_step = 0.1f;

/**
 * @brief USART6 统一格式化输出，所有蓝牙串口发送都走这一个出口
 * @param fmt  printf 格式字符串
 * @param ...  可变参数
 */
void App_Uart6Printf(const char *fmt, ...)
{
    char tx_buf[128];
    va_list args;
    int tx_len;

    va_start(args, fmt);
    tx_len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    if ((tx_len > 0) && ((uint32_t)tx_len < sizeof(tx_buf))) {
        (void)HAL_UART_Transmit(&huart6, (uint8_t *)tx_buf, (uint16_t)tx_len, 100U);
    }
}

/**
 * @brief 接收串口命令，中断里只存入标志位，不做任何耗时操作
 */
void App_CommandUartRxByte(uint8_t data)
{
    s_data = data;
}

/**
 * @brief 处理蓝牙串口命令（在任务上下文中调用，不可在中断里调用）
 *        读取 s_data 标志位，执行对应的电机控制或 PID 调节，
 *        处理完后将 s_data 清零。
 */
void App_ProcessCommand(void)
{
    uint8_t data;

    data = s_data;
    if (data == 0U) {
        return;
    }
    s_data = 0U;

    if (data == 'a' || data == 'A') {
//        Emm_V5_Pos_Control_ByPulse(5, 0, 100, 50, 100.0f, 1, false);
//        osDelay(570);
//        Emm_V5_Pos_Control_ByPulse(5, 1, 100, 50, 100.0f, 1, false);
//        osDelay(1000);
//        Emm_V5_Pos_Control_ByPulse(5, 0, 100, 50, 50.0f, 1, false);
//        osDelay(1000);
//        Emm_V5_Pos_Control_ByPulse(5, 0, 100, 20, 1.2f, 1, false);
    } else if (data == 's' || data == 'S') {
        Emm_V5_Pos_Control_ByPulse(5, 0, 50, 0, 100.0f, 1, false);
    } else if (data == 't' || data == 'T') {
        Emm_V5_Trigger_Zero(5, EMM_V5_ZERO_SINGLE_NEAREST, false);
    } else if (data == 'P') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, s_pid_step, 0.0f, 0.0f);
        App_Uart6Printf("pos KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd,
                        s_gangzhu_pid.q_pid.Pout, s_gangzhu_pid.q_pid.Iout,
                        s_gangzhu_pid.q_pid.Dout);
    } else if (data == 'p') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, -s_pid_step, 0.0f, 0.0f);
        App_Uart6Printf("pos KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd,
                        s_gangzhu_pid.q_pid.Pout, s_gangzhu_pid.q_pid.Iout,
                        s_gangzhu_pid.q_pid.Dout);
    } else if (data == 'I') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, s_pid_step, 0.0f);
        App_Uart6Printf("pos KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd,
                        s_gangzhu_pid.q_pid.Pout, s_gangzhu_pid.q_pid.Iout,
                        s_gangzhu_pid.q_pid.Dout);
    } else if (data == 'i') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, -s_pid_step, 0.0f);
        App_Uart6Printf("pos KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd,
                        s_gangzhu_pid.q_pid.Pout, s_gangzhu_pid.q_pid.Iout,
                        s_gangzhu_pid.q_pid.Dout);
    } else if (data == 'D') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, 0.0f, s_pid_step);
        App_Uart6Printf("pos KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd,
                        s_gangzhu_pid.q_pid.Pout, s_gangzhu_pid.q_pid.Iout,
                        s_gangzhu_pid.q_pid.Dout);
    } else if (data == 'd') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, 0.0f, -s_pid_step);
        App_Uart6Printf("pos KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd,
                        s_gangzhu_pid.q_pid.Pout, s_gangzhu_pid.q_pid.Iout,
                        s_gangzhu_pid.q_pid.Dout);
    } else if (data == 'Q') {
        /* 切换 PID 调试步距: 1.0 -> 0.1 -> 0.01 -> 1.0 */
        if (s_pid_step >= 0.9f) {
            s_pid_step = 0.1f;
        } else if (s_pid_step >= 0.09f) {
            s_pid_step = 0.01f;
        } else {
            s_pid_step = 1.0f;
        }
        App_Uart6Printf("step=%.2f\r\n", s_pid_step);
    } else if (data == 'J') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, s_pid_step, 0.0f, 0.0f);
        App_Uart6Printf("spd KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.speed_pid.Kp, s_gangzhu_pid.speed_pid.Ki,
                        s_gangzhu_pid.speed_pid.Kd,
                        s_gangzhu_pid.speed_pid.Pout, s_gangzhu_pid.speed_pid.Iout,
                        s_gangzhu_pid.speed_pid.Dout);
    } else if (data == 'j') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, -s_pid_step, 0.0f, 0.0f);
        App_Uart6Printf("spd KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.speed_pid.Kp, s_gangzhu_pid.speed_pid.Ki,
                        s_gangzhu_pid.speed_pid.Kd,
                        s_gangzhu_pid.speed_pid.Pout, s_gangzhu_pid.speed_pid.Iout,
                        s_gangzhu_pid.speed_pid.Dout);
    } else if (data == 'K') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, s_pid_step, 0.0f);
        App_Uart6Printf("spd KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.speed_pid.Kp, s_gangzhu_pid.speed_pid.Ki,
                        s_gangzhu_pid.speed_pid.Kd,
                        s_gangzhu_pid.speed_pid.Pout, s_gangzhu_pid.speed_pid.Iout,
                        s_gangzhu_pid.speed_pid.Dout);
    } else if (data == 'k') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, -s_pid_step, 0.0f);
        App_Uart6Printf("spd KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.speed_pid.Kp, s_gangzhu_pid.speed_pid.Ki,
                        s_gangzhu_pid.speed_pid.Kd,
                        s_gangzhu_pid.speed_pid.Pout, s_gangzhu_pid.speed_pid.Iout,
                        s_gangzhu_pid.speed_pid.Dout);
    } else if (data == 'L') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, 0.0f, s_pid_step);
        App_Uart6Printf("spd KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.speed_pid.Kp, s_gangzhu_pid.speed_pid.Ki,
                        s_gangzhu_pid.speed_pid.Kd,
                        s_gangzhu_pid.speed_pid.Pout, s_gangzhu_pid.speed_pid.Iout,
                        s_gangzhu_pid.speed_pid.Dout);
    } else if (data == 'l') {
        GangzhuPid_AdjustSpeedGains(&s_gangzhu_pid, 0.0f, 0.0f, -s_pid_step);
        App_Uart6Printf("spd KP=%.2f KI=%.2f KD=%.2f P=%.3f I=%.3f D=%.3f\r\n",
                        s_gangzhu_pid.speed_pid.Kp, s_gangzhu_pid.speed_pid.Ki,
                        s_gangzhu_pid.speed_pid.Kd,
                        s_gangzhu_pid.speed_pid.Pout, s_gangzhu_pid.speed_pid.Iout,
                        s_gangzhu_pid.speed_pid.Dout);
    } else if (data == 'z') {
        s_gangzhu_pid.speed_enabled = !s_gangzhu_pid.speed_enabled;
        App_Uart6Printf("spd_en=%d target=%.2f\r\n",
                        s_gangzhu_pid.speed_enabled,
                        s_gangzhu_pid.target_speed);
    } else if (data == 'W') {
        s_gangzhu_pid.target_speed += s_pid_step;
        App_Uart6Printf("spd_en=%d target=%.2f\r\n",
                        s_gangzhu_pid.speed_enabled,
                        s_gangzhu_pid.target_speed);
    } else if (data == 'w') {
        s_gangzhu_pid.target_speed -= s_pid_step;
        App_Uart6Printf("spd_en=%d target=%.2f\r\n",
                        s_gangzhu_pid.speed_enabled,
                        s_gangzhu_pid.target_speed);
    }
}

/**
 * @brief PID 日志开关，调试用
 * @return true 开启日志输出，false 关闭
 */
bool App_IsPidLogEnabled(void)
{
    return true;
}

/**
 * @brief 初始化应用层调试状态
 */
void App_Init(void)
{
    s_data = 0U;
}
