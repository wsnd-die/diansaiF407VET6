#include "navigation.h"
#include "bujin.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define NAV_PI                    3.1415926f

/* 旋转对齐控制 PID 及前馈参数 */
#define ALIGN_KP                  2.8f          /**< 旋转对齐状态比例系数 */
#define ALIGN_KD                  0.3f          /**< 旋转对齐状态微分系数 */
#define ALIGN_FF_BASE             0.6f          /**< 旋转对齐状态基础静态摩擦力前馈控制量 */
#define ALIGN_FF_THRESH           0.15f         /**< 前馈启动阈值（当角度偏差大于此值时使用前馈） */
#define MAX_ANGULAR               1.2f          /**< 旋转对齐状态下最大角速度限制 (rad/s) */
#define ALIGN_ERR_THRESH          0.03f         /**< 对齐精度判定阈值 (rad) */

/* 直线行进控制参数 */
#define MOVE_LINEAR_SPEED         300.0f        /**< 直线行进最大期望线速度 (mm/s) */
#define MOVE_ANGULAR_KP           1.5f          /**< 纠偏角速度比例系数 */
#define MOVE_ANGULAR_KD           0.1f          /**< 纠偏角速度微分系数 */
#define MOVE_ARRIVE_DIST          10.0f         /**< 目标点判定范围半径，小于 20mm 认为到达 (mm) */
#define MOVE_MIN_LINEAR           20.0f         /**< 减速时最小保证线速度 (mm/s) */
#define MOVE_MAX_ANGULAR          0.8f          /**< 直线纠偏中最大角速度限制 (rad/s) */

/* 到达最终角度调整控制参数 */
#define ARRIVED_ANGULAR_KP        2.0f          /**< 终点旋转比例系数 */
#define ARRIVED_ANGULAR_KD        0.1f          /**< 终点旋转微分系数 */
#define ARRIVED_MAX_ANGULAR       0.6f          /**< 终点最大角速度限制 (rad/s) */
#define ARRIVED_ERR_THRESH        0.02f         /**< 最终角度对齐允许最大误差 (rad) */

/* 状态机全局变量 */
Navigation_State_t navigation_state = NAVIGATION_STATE_IDLE;
volatile position_t g_robot_pos = {0.0f, 0.0f, 0.0f};           /* 机器人在世界坐标系下的绝对位姿 */
struct move speed = {0.0f, 0.0f, 0.0f};                         /* 保留以防其他文件 extern */
struct move angle_speed = {0.0f, 0.0f, 0.0f};
float nav_yaw_zero_deg=0.0f;
int TarAngle;
float TarPos = 360.0f;
bool is_moving;
float angle_fix;
float v[2];                                                     /* 保存计算得到的左右侧轮线速度，单位：mm/s */

static position_t target;                                       /* 当前的导航目标位姿 */
static position_t start;                                        /* 启动本次导航时的机器人位姿 */

/* 静态控制函数声明 */
static void Navigation_HandleIdle(void);
static void Navigation_HandleTargetAlign(void);
static void Navigation_HandleMoving(void);
static void Navigation_HandleArrived(void);
static float Navigation_NormalizeRad(float angle);

/**
 * @brief 规范化角度到 [-180, 180] 度范围内
 */
float Navigation_NormalizeDeg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 复位里程计与朝向角零偏
 */
void Navigation_Reset(float start_x_mm, float start_y_mm, float yaw_zero_deg)
{
    g_robot_pos.x = start_x_mm;
    g_robot_pos.y = start_y_mm;
    g_robot_pos.yaw = 0.0f;
    nav_yaw_zero_deg = yaw_zero_deg;
    navigation_state = NAVIGATION_STATE_IDLE;
    /* 重启/复位初始化时，主动下发一次速度 0，强制底盘电机静止，防止重启后电机自转 */
    Chassis_SetSpeed(0.0f, 0.0f);
}

/**
 * @brief 根据位移差更新机器人二维坐标
 * @note  由于机器人在大地坐标系下的航向偏角定义，采用 sin(yaw) 更新 x，cos(yaw) 更新 y
 */
void Navigation_UpdateByDelta(float delta_mm, float yaw_deg)
{
    float yaw_rad = yaw_deg * NAV_PI / 180.0f;

    g_robot_pos.yaw = yaw_deg;
    g_robot_pos.x += delta_mm * sinf(yaw_rad);
    g_robot_pos.y += delta_mm * cosf(yaw_rad);
}

/**
 * @brief 获取当前的偏航角度
 */
float Navigation_GetYawDeg(void)
{
    return g_robot_pos.yaw;
}

/**
 * @brief 周期调度主函数，根据当前状态机执行不同的动作
 */
void Navigation_TaskTick(void)
{
    switch (navigation_state) {
    case NAVIGATION_STATE_IDLE:
        Navigation_HandleIdle();
        break;
    case NAVIGATION_STATE_TARGET_ALIGN:
        Navigation_HandleTargetAlign();
        break;
    case NAVIGATION_STATE_MOVING:
        Navigation_HandleMoving();
        break;
    case NAVIGATION_STATE_ARRIVED:
        Navigation_HandleArrived();
        break;
    default:
        navigation_state = NAVIGATION_STATE_IDLE;
        break;
    }
}

/**
 * @brief 发送导航任务请求
 */
int8_t Navigation_Request(float target_x_mm, float target_y_mm, float target_yaw_rad)
{
    /* 如果当前正在执行其他导航任务，直接拒绝 */
    if (navigation_state != NAVIGATION_STATE_IDLE) {
        return -1;
    }

    target.x = target_x_mm;
    target.y = target_y_mm;
    target.yaw = target_yaw_rad;
    start = g_robot_pos;
    /* 触发状态机，第一步进行朝向目标点的对齐旋转 */
    navigation_state = NAVIGATION_STATE_TARGET_ALIGN;
    return 0;
}

/**
 * @brief 查询导航状态是否空闲
 */
bool Navigation_IsIdle(void)
{
    return navigation_state == NAVIGATION_STATE_IDLE;
}

/**
 * @brief 强制关闭状态机并将底盘轮子停转
 */
void Navigation_Stop(void)
{
    navigation_state = NAVIGATION_STATE_IDLE;
    Chassis_SetSpeed(0.0f, 0.0f);
}

/**
 * @brief 闲置状态处理：重置期望速度
 */
static void Navigation_HandleIdle(void)
{
    speed.real = 0.0f;
    angle_speed.real = 0.0f;
}

/**
 * @brief 原地旋转调整朝向，使机器人正前方向指向目标点
 */
static void Navigation_HandleTargetAlign(void)
{
    static Navigation_State_t last_state = NAVIGATION_STATE_IDLE;
    static float target_yaw;
    static float last_err;
    static TickType_t last_time;
    float err;
    float dt;
    float angular_speed;
    TickType_t now;

    /* 首次进入该状态时，计算两点之间的绝对方位角作为期望旋转朝向 */
    if (last_state != NAVIGATION_STATE_TARGET_ALIGN) {
        target_yaw = atan2f(target.x - start.x, target.y - start.y);
        last_err = 0.0f;
        last_time = xTaskGetTickCount();
    }
    last_state = NAVIGATION_STATE_TARGET_ALIGN;

    /* 偏差角度 = 期望角(rad) - 当前角度(rad) */
    err = Navigation_NormalizeRad(target_yaw - g_robot_pos.yaw * PI / 180.0f);
    
    /* 偏差角度小于判定门限，说明朝向已对准目标点，进入直线行进状态 */
    if (fabsf(err) < ALIGN_ERR_THRESH) {
        navigation_state = NAVIGATION_STATE_MOVING;
        last_state = NAVIGATION_STATE_IDLE;
        return;
    }

    now = xTaskGetTickCount();
    dt = (float)(now - last_time) / (float)configTICK_RATE_HZ;//计算时间间隔，单位：秒
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    /* PD闭环反馈控制旋转 */
    angular_speed = -(ALIGN_KP * err + ALIGN_KD * (err - last_err) / dt);
    
    /* 引入前馈常数，以克服电机及地面的死区摩擦力，提高调节速度 */
    if (fabsf(err) > ALIGN_FF_THRESH) {
        angular_speed -= (err > 0.0f) ? ALIGN_FF_BASE : -ALIGN_FF_BASE;
    }
    
    /* 饱和度限幅保护 */
    if (angular_speed > MAX_ANGULAR) {
        angular_speed = MAX_ANGULAR;
    } else if (angular_speed < -MAX_ANGULAR) {
        angular_speed = -MAX_ANGULAR;
    }

    /* 旋转状态：线速度为0，输出期望角速度 */
    Chassis_SetSpeed(0.0f, angular_speed);
    last_err = err;
    last_time = now;
}

/**
 * @brief 直线行进处理函数：直线行进的同时进行航偏纠偏控制
 */
static void Navigation_HandleMoving(void)
{
    static Navigation_State_t last_state = NAVIGATION_STATE_IDLE;
    static float last_err;
    static TickType_t last_time;
    float dx = target.x - g_robot_pos.x;
    float dy = target.y - g_robot_pos.y;
    float distance = sqrtf(dx * dx + dy * dy);   /* 计算距离目标点的欧氏距离 */
    float err;
    float dt;
    float angular_speed;
    float linear_speed = MOVE_LINEAR_SPEED;
    TickType_t now;

    /* 到达目标点判定半径内，说明行进完成，进入终点角度微调状态 */
    if (distance < MOVE_ARRIVE_DIST) {
        navigation_state = NAVIGATION_STATE_ARRIVED;
        Chassis_SetSpeed(0.0f, 0.0f);
        last_state = NAVIGATION_STATE_IDLE;
        return;
    }

    if (last_state != NAVIGATION_STATE_MOVING) {
        float init_dx = target.x - g_robot_pos.x;
        float init_dy = target.y - g_robot_pos.y;
        last_err = Navigation_NormalizeRad(atan2f(init_dx, init_dy) - g_robot_pos.yaw * PI / 180.0f);
        last_time = xTaskGetTickCount();
    }
    last_state = NAVIGATION_STATE_MOVING;

    /* 纠偏误差：根据当前位置和目标点连线的方位角，对比当前机器人的朝向角度 */
    err = Navigation_NormalizeRad(atan2f(dx, dy) - g_robot_pos.yaw * PI / 180.0f);
    now = xTaskGetTickCount();
    dt = (float)(now - last_time) / (float)configTICK_RATE_HZ;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    /* PD计算纠偏输出的角速度 */
    angular_speed = -(MOVE_ANGULAR_KP * err + MOVE_ANGULAR_KD * (err - last_err) / dt);
    if (angular_speed > MOVE_MAX_ANGULAR) {
        angular_speed = MOVE_MAX_ANGULAR;
    } else if (angular_speed < -MOVE_MAX_ANGULAR) {
        angular_speed = -MOVE_MAX_ANGULAR;
    }
    
    /* 临近终点减速逻辑，距离小于 200mm 时，线速度呈线性比例减小 */
    if (distance < 200.0f) {
        linear_speed = MOVE_LINEAR_SPEED * distance / 200.0f;
        if (linear_speed < MOVE_MIN_LINEAR) {
            linear_speed = MOVE_MIN_LINEAR; /* 限制最小速度，防止死区卡住 */
        }
    }

    Chassis_SetSpeed(linear_speed, angular_speed);
    last_err = err;
    last_time = now;
}

/**
 * @brief 终点调整状态处理：在目标点原地旋转至最终期望的偏航角
 */
static void Navigation_HandleArrived(void)
{
    static Navigation_State_t last_state = NAVIGATION_STATE_IDLE;
    static float last_err;
    static TickType_t last_time;
    float err = Navigation_NormalizeRad(target.yaw - g_robot_pos.yaw * PI / 180.0f);
    float dt;
    float angular_speed;
    TickType_t now;

    /* 朝向角误差小于允许误差，本次导航宣告圆满结束，停止小车并进入空闲 */
    if (fabsf(err) < ARRIVED_ERR_THRESH) {
        Navigation_Stop();
        last_state = NAVIGATION_STATE_IDLE;
        return;
    }

    if (last_state != NAVIGATION_STATE_ARRIVED) {
        last_err = 0.0f;
        last_time = xTaskGetTickCount();
    }
    last_state = NAVIGATION_STATE_ARRIVED;

    now = xTaskGetTickCount();
    dt = (float)(now - last_time) / (float)configTICK_RATE_HZ;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }
    
    /* PD计算旋转调整的角速度 */
    angular_speed = -(ARRIVED_ANGULAR_KP * err + ARRIVED_ANGULAR_KD * (err - last_err) / dt);
    if (angular_speed > ARRIVED_MAX_ANGULAR) {
        angular_speed = ARRIVED_MAX_ANGULAR;
    } else if (angular_speed < -ARRIVED_MAX_ANGULAR) {
        angular_speed = -ARRIVED_MAX_ANGULAR;
    }
    Chassis_SetSpeed(0.0f, angular_speed);
    last_err = err;
    last_time = now;
}

/**
 * @brief 设置底盘的全局速度（线速度与角速度）
 * @note  根据差速机器人运动学公式：
 *        v_left  = v_linear + w * half_track
 *        v_right = v_linear - w * half_track
 *        然后根据轮子半径转换为对应电机的物理转速(RPM)。
 */
void Chassis_SetSpeed(float linear_vel_mm_s, float angular_vel_rad_s)
{
    float left_vel = linear_vel_mm_s + angular_vel_rad_s * HALF_TRACK_MM;
    float right_vel = linear_vel_mm_s - angular_vel_rad_s * HALF_TRACK_MM;
    
    /* 物理RPM计算式：rpm = v / (2 * pi * r) * 60 */
    float left_rpm = fabsf(left_vel) / (TWO_PI * WHEEL_RADIUS_MM) * 60.0f;
    float right_rpm = fabsf(right_vel) / (TWO_PI * WHEEL_RADIUS_MM) * 60.0f;

    v[0] = left_vel;
    v[1] = right_vel;
    
    /* 控制下发：方向参数由速度符号决定，转速精度转换为整数，同时下发并设置同步标志 */
    Emm_V5_Vel_Control(left_head, left_vel >= 0.0f, (uint16_t)(left_rpm + 0.5f), 255, true);
    Emm_V5_Vel_Control(left_tail, left_vel >= 0.0f, (uint16_t)(left_rpm + 0.5f), 255, true);
    Emm_V5_Vel_Control(right_head, right_vel >= 0.0f, (uint16_t)(right_rpm + 0.5f), 255, true);
    Emm_V5_Vel_Control(right_tail, right_vel >= 0.0f, (uint16_t)(right_rpm + 0.5f), 255, true);
    
    /* 广播/通知，触发多电机硬件同步对齐运动 */
    Emm_V5_Synchronous_motion(0);
}

/**
 * @brief 规范化角度到 [-PI, PI] 弧度区间
 */
static float Navigation_NormalizeRad(float angle)
{
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}

