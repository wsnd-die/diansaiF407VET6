#ifndef __APP_H
#define __APP_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 应用程序运行模式枚举
 */
typedef enum {
    APP_MODE_IDLE,      /**< 空闲模式：机器人处于静止状态，不执行任何航线 */
    APP_MODE_TEST,      /**< 测试模式：用于调试和测试功能，可能包含自定义的测试逻辑 */
    APP_MODE_ROUTE_A,   /**< 航线 A 模式：执行第一阶段的路径导航（例如向深处前行） */
    APP_MODE_ROUTE_B,   /**< 航线 B 模式：预留模式 */
    APP_MODE_ROUTE_C    /**< 航线 C 模式：执行第二阶段的路径导航（例如折返或区域内作业） */
} AppMode_t;

/**
 * @brief 路径点（航点）结构体
 */
typedef struct {
    float x_mm;         /**< 目标点 X 坐标，单位：毫米 */
    float y_mm;         /**< 目标点 Y 坐标，单位：毫米 */
    float yaw_rad;      /**< 目标点朝向角，单位：弧度 */
    bool has_action;    /**< 是否在到达目标点后执行舵机动作 (true/false) */
} AppWaypoint_t;

/* 全局应用模式变量，由导航任务或串口控制修改 */
extern volatile AppMode_t g_app_mode;

/**
 * @brief 初始化应用状态，复位模式和控制变量
 */
void App_Init(void);

/**
 * @brief 运行当前应用模式下的任务逻辑，根据当前模式依次执行航线
 */
void App_RunCurrentMode(void);

/**
 * @brief 设置应用程序的目标运行模式
 * @param mode 目标模式
 */
void App_SetMode(AppMode_t mode);

/**
 * @brief 查询当前应用是否正在运行路径导航
 * @return true 正在运行，false 处于空闲或停止状态
 */
bool App_IsRunning(void);

/**
 * @brief 串口指令接收处理函数，用于解析外部启动/停止指令
 * @param data 接收到的串口数据字节 ('a'/'A' 启动, 't'/'T' 停止)
 */
void App_CommandUartRxByte(uint8_t data);

#endif

