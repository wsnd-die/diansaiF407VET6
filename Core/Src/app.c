#include "app.h"
#include "navigation.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
/* 定义 PI 常量，避免未定义标识符 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 获取航线数组的元素个数 */
#define APP_ROUTE_LEN(route) ((uint8_t)(sizeof(route) / sizeof((route)[0])))
/* 方便定义路径点（X_mm, Y_mm, Yaw_rad）的辅助宏 */
#define WAYPOINT(x, y, yaw)  {(x), (y), (yaw)}

/* 当前系统的全局应用模式 */
volatile AppMode_t g_app_mode = APP_MODE_IDLE;

/* 内部状态变量：路径导航是否运行中，是否收到停止请求 */
static volatile bool s_app_running;
static volatile bool s_stop_requested;

/* 航线 A 的目标路径点序列 */
static const AppWaypoint_t k_route_a[] = {
    WAYPOINT(0.0f, 750.0f, 0.0f),
    WAYPOINT(0.0f, 1250.0f, 0.0f),
    WAYPOINT(0.0f, 1750.0f, 0.0f),
    WAYPOINT(0.0f, 2250.0f, 0.0f),
    WAYPOINT(0.0f, 0.0f, -PI / 2.0f),
};

/* 航线 C 的目标路径点序列 */
static const AppWaypoint_t k_route_c[] = {
    WAYPOINT(-1900.0f, 0.0f, 0.0f),
    WAYPOINT(-1900.0f, 450.0f, 0.0f),
    WAYPOINT(-1900.0f, 950.0f, 0.0f),
    WAYPOINT(-1900.0f, 1450.0f, 0.0f),
    WAYPOINT(-1900.0f, 1950.0f, 0.0f),
    WAYPOINT(-1900.0f, 2450.0f, 0.0f),
    WAYPOINT(-2700.0f, 2450.0f, -PI),
    WAYPOINT(-2700.0f, 1950.0f, -PI),
    WAYPOINT(-2700.0f, 1450.0f, -PI),
    WAYPOINT(-2700.0f, 950.0f, -PI),
    WAYPOINT(-2700.0f, 450.0f, -PI),
    WAYPOINT(-2700.0f, 0.0f, -PI),
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
 * @brief 接收串口命令，用于启动或停止导航任务
 * @param data 接收到的字符。'a'/'A' 代表启动航线 A，'t'/'T' 代表立即紧急停机
 */
void App_CommandUartRxByte(uint8_t data)
{
    if (data == 'a' || data == 'A') {
        s_app_running = true;
        s_stop_requested = false;
        App_SetMode(APP_MODE_ROUTE_A);
    } else if (data == 't' || data == 'T') {
        s_app_running = false;
        s_stop_requested = true;
        App_SetMode(APP_MODE_IDLE);
    }
}
static long PC_Comm_FloatToCenti(float value)
{
    if (value >= 0.0f) {
        return (long)(value * 100.0f + 0.5f);
    }

    return (long)(value * 100.0f - 0.5f);
}


void PC_Comm_SendNavigationStatus(void)
{
    char tx_buf[96];
    int len;
    long yaw = PC_Comm_FloatToCenti(Navigation_GetYawDeg());
    long x = PC_Comm_FloatToCenti(g_robot_pos.x);
    long y = PC_Comm_FloatToCenti(g_robot_pos.y);
    long yaw_abs = (yaw < 0) ? -yaw : yaw;
    long x_abs = (x < 0) ? -x : x;
    long y_abs = (y < 0) ? -y : y;

    len = snprintf(tx_buf, sizeof(tx_buf),
                   "yaw=%s%ld.%02ld,x=%s%ld.%02ld,y=%s%ld.%02ld,state=%u,test=%u\r\n",
                   (yaw < 0) ? "-" : "", yaw_abs / 100L, yaw_abs % 100L,
                   (x < 0) ? "-" : "", x_abs / 100L, x_abs % 100L,
                   (y < 0) ? "-" : "", y_abs / 100L, y_abs % 100L,
                   (unsigned int)navigation_state,
                   (unsigned int)s_app_running);

    if (len > 0) {
        if (len >= (int)sizeof(tx_buf)) {
            len = (int)sizeof(tx_buf) - 1;
        }
        (void)HAL_UART_Transmit(&huart6, (uint8_t *)tx_buf, (uint16_t)len, 50);
    }
}
/**
 * @brief 应用主循环/任务中调用的模式执行函数
 *        根据当前所处的 g_app_mode 决定执行哪条航线，或处理停止请求
 */
void App_RunCurrentMode(void)
{
    if (s_stop_requested) {
        s_stop_requested = false;
        Navigation_Stop();
        return;
    }

    if (g_app_mode == APP_MODE_ROUTE_A) {
        /* 执行航线 A，执行完毕后自动进入模式 C */
        App_RunRoute(k_route_a, APP_ROUTE_LEN(k_route_a), APP_MODE_ROUTE_C);
    } else if (g_app_mode == APP_MODE_ROUTE_C) {
        /* 执行航线 C，执行完毕后回到空闲状态 */
        App_RunRoute(k_route_c, APP_ROUTE_LEN(k_route_c), APP_MODE_IDLE);
    }
}

/**
 * @brief 顺序执行一个航线数组中的所有点
 * @param route 航点数组指针
 * @param route_len 航点数组长度
 * @param next_mode 执行完成后的下一个应用模式
 */
static void App_RunRoute(const AppWaypoint_t *route, uint8_t route_len,
                         AppMode_t next_mode)
{
    uint8_t i;

    /* 逐个航点遍历，确保系统仍处于运行状态 */
    for (i = 0U; i < route_len && App_IsRunning(); i++) {
        /* 请求导航到当前航点坐标 */
        (void)Navigation_Request(route[i].x_mm, route[i].y_mm, route[i].yaw_rad);

        /* 等待导航模块到达目标点（进入闲置状态），且保证系统未被外部中止 */
        while (!Navigation_IsIdle() && App_IsRunning()) {
            vTaskDelay(pdMS_TO_TICKS(50)); /* 延时 50ms */
        }
    }

    /* 如果中途被停止，直接退出 */
    if (!App_IsRunning()) {
        return;
    }

    /* 如果下一步是空闲模式，重置运行状态并停止底盘 */
    if (next_mode == APP_MODE_IDLE) {
        s_app_running = false;
        Navigation_Stop();
    }

    /* 切换到下一个运行模式 */
    App_SetMode(next_mode);
}

