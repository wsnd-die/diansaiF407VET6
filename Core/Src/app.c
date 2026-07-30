#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "bujin.h"
#include "gangzhu_pid.h"
#include <stdarg.h>
#include <stdio.h>
/* 定义 PI 常量，避免未定义标识符 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 获取航线数组的元素个数 */
#define APP_ROUTE_LEN(route) ((uint8_t)(sizeof(route) / sizeof((route)[0])))
/* 方便定义路径点（X_mm, Y_mm, Yaw_rad, has_action）的辅助宏 */
#define WAYPOINT(x, y, yaw, act)    {(x), (y), (yaw), (act)}
#define WAYPOINT_NO_ACT(x, y, yaw)  {(x), (y), (yaw), false}
#define WAYPOINT_ACT(x, y, yaw)     {(x), (y), (yaw), true}

/* 当前系统的全局应用模式 */
volatile AppMode_t g_app_mode = APP_MODE_IDLE;

/* 内部状态变量：路径导航是否运行中，是否收到停止请求 */
static volatile bool s_app_running;
static volatile bool s_stop_requested;
static volatile uint8_t s_data;

/* 航线 A 的目标路径点序列 */
static const AppWaypoint_t k_route_a[] = {
    WAYPOINT(0.0f, 700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1200.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 2200.0f, 0.0f, 1),
    WAYPOINT(0.0f, 0.0f, 0.0f, 0),
    WAYPOINT(-1950.0f, 0.0f, 0.0f, false),
};

/* 航线 C 的目标路径点序列 */
static const AppWaypoint_t k_route_c[] = {
    WAYPOINT(-1950.0f, 0.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 500.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 1000.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 1500.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 2000.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 2500.0f, 0.0f, false),
    WAYPOINT(-2700.0f, 2500.0f, -PI, false),
    WAYPOINT(-2700.0f, 2000.0f, -PI, false),
    WAYPOINT(-2700.0f, 1500.0f, -PI, false),
    WAYPOINT(-2700.0f, 1000.0f, -PI, false),
    WAYPOINT(-2700.0f, 500.0f, -PI, false),
    WAYPOINT(-2700.0f, 0.0f, -PI, false),
};

/* 内部静态函数：执行特定的一组航线点，并跳转到指定的下一个模式 */
static void App_RunRoute(const AppWaypoint_t *route, uint8_t route_len,
                         AppMode_t next_mode);

/**
 * @brief 初始化应用层状态
 */
void App_Init(void)
{
    g_app_mode = APP_MODE_IDLE;
    s_app_running = false;
    s_stop_requested = false;
    s_data = 0U;
}

/**
 * @brief 设置目标应用模式
 */
void App_SetMode(AppMode_t mode)
{
    g_app_mode = mode;
}

/**
 * @brief 查询导航状态
 */
bool App_IsRunning(void)
{
    return s_app_running;
}

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
//         Emm_V5_Pos_Control_ByPulse(5, 0, 100, 50, 100.0f, 1, false);
//           osDelay(565);
//      Emm_V5_Pos_Control_ByPulse(5, 1, 100, 50, 100.0f, 1, false);
//        osDelay(980);
//       Emm_V5_Pos_Control_ByPulse(5, 0, 100, 50, 100.0f, 1, false);
//        
//           osDelay(650);
//         Emm_V5_Pos_Control_ByPulse(5, 1, 100, 10,1.0f, 1, false);
        //成功过的版本
        Emm_V5_Pos_Control_ByPulse(5, 0, 100, 50, 100.0f, 1, false);
           osDelay(570);
      Emm_V5_Pos_Control_ByPulse(5, 1, 100, 50, 100.0f, 1, false);
        osDelay(1000);
       Emm_V5_Pos_Control_ByPulse(5, 0, 100, 50, 50.0f, 1, false);
        
           osDelay(1000);
         Emm_V5_Pos_Control_ByPulse(5, 0, 100, 20, 1.2f, 1, false);
        
//        Emm_V5_Pos_Control_ByPulse(5, 0, 100, 50, 20.0f, 1, false);
//         osDelay(200);
//        HAL_Delay(100);
//         Emm_V5_Pos_Control_ByPulse(5, 1, 100, 50, 50.0f, 1, false);
    } else if (data == 's' || data == 'S') {
          Emm_V5_Pos_Control_ByPulse(5, 0, 50, 0, 100.0f, 1, false);
    } else if (data == 't' || data == 'T') {
        Emm_V5_Trigger_Zero(5, EMM_V5_ZERO_SINGLE_NEAREST, false);
    } else if (data == 'P') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.05f, 0.0f, 0.0f);
        App_Uart6Printf("out: KP=%.2f,KI=%.2f,KD=%.2f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd);
    } else if (data == 'p') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, -0.05f, 0.0f, 0.0f);
        App_Uart6Printf("out: KP=%.2f,KI=%.2f,KD=%.2f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd);
    } else if (data == 'I') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, 0.01f, 0.0f);
        App_Uart6Printf("out: KP=%.2f,KI=%.2f,KD=%.2f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd);
    } else if (data == 'i') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, -0.01f, 0.0f);
        App_Uart6Printf("out: KP=%.2f,KI=%.2f,KD=%.2f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd);
    } else if (data == 'D') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, 0.0f, 0.01f);
        App_Uart6Printf("out: KP=%.2f,KI=%.2f,KD=%.2f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd);
    } else if (data == 'd') {
        GangzhuPid_AdjustGains(&s_gangzhu_pid, 0.0f, 0.0f, -0.01f);
        App_Uart6Printf("out: KP=%.2f,KI=%.2f,KD=%.2f\r\n",
                        s_gangzhu_pid.kp, s_gangzhu_pid.ki, s_gangzhu_pid.kd);
    }
}

