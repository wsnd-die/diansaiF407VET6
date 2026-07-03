#include "odometer.h"
#include "bujin.h"
#include "hwt101_hal.h"
#include "navigation.h"

extern UART_HandleTypeDef huart2;

/* 里程计读取 5 号步进驱动器，和当前步进控制代码保持一致。 */
#define ODOMETER_MOTOR_ADDR              3

/*
 * 轮子/卷轮半径，单位 cm。
 */
#define ODOMETER_WHEEL_RADIUS_CM         0.85f
#define ODOMETER_PI                      3.1415926f
#define ODOMETER_DEG_TO_CM               (2.0f * ODOMETER_PI * ODOMETER_WHEEL_RADIUS_CM / 360.0f)

/*
 * 假设 Emm V5 读取当前位置 0x36 回帧：
 * addr + 0x36 + sign + angle[4] + 0x6B
 * 65536 个位置值表示一圈，所以角度 = raw * 360 / 65536。
 */
#define ODOMETER_FRAME_MAX_LEN           8
#define ODOMETER_POS_CMD                 0x36
#define ODOMETER_FRAME_END               0x6B
#define ODOMETER_ANGLE_SCALE             (360.0f / 65536.0f)

volatile float g_odometer_distance_cm = 0.0f;
volatile float g_odometer_delta_cm = 0.0f;
volatile float g_odometer_motor_angle_deg = 0.0f;

static uint8_t odom_rx_buf[ODOMETER_FRAME_MAX_LEN];
static uint8_t odom_rx_len = 0;
static uint8_t odom_has_last_angle = 0;
static float odom_last_angle_deg = 0.0f;

static void Odometer_UpdateByAngle(float angle_deg)
{
    float delta_deg;
    float delta_cm;

    g_odometer_motor_angle_deg = angle_deg;

    if (odom_has_last_angle == 0)
    {
        odom_last_angle_deg = angle_deg;
        odom_has_last_angle = 1;
        return;
    }

    delta_deg = angle_deg - odom_last_angle_deg;
    odom_last_angle_deg = angle_deg;

    if (delta_deg > 180.0f)
        {
            delta_deg -= 360.0f;
        }
    else if (delta_deg < -180.0f)
    {
             delta_deg += 360.0f;
    }

    /* 有符号位移 = 电机角度变化 / 360 * 轮子周长；倒车时 delta_cm 为负。 */
    delta_cm = delta_deg * ODOMETER_DEG_TO_CM;
    g_odometer_delta_cm = delta_cm;
    g_odometer_distance_cm += delta_cm;
    Navigation_UpdateByDelta(delta_cm, HWT101_GetYaw());
}

static void Odometer_ParseFrame(void)//最终使用的函数
{
    uint8_t sign;
    uint32_t raw_angle;
    float angle_deg;

    if (odom_rx_len != ODOMETER_FRAME_MAX_LEN)
    {
        return;
    }

    sign = odom_rx_buf[2];
    raw_angle = ((uint32_t)odom_rx_buf[3] << 24) |
                ((uint32_t)odom_rx_buf[4] << 16) |
                ((uint32_t)odom_rx_buf[5] << 8)  |
                ((uint32_t)odom_rx_buf[6] << 0);

    angle_deg = (float)raw_angle * ODOMETER_ANGLE_SCALE;
    if (sign != 0)
    {
        angle_deg = -angle_deg;
    }

    Odometer_UpdateByAngle(angle_deg);
}

void Odometer_Init(void)
{
    g_odometer_distance_cm = 0.0f;
    g_odometer_delta_cm = 0.0f;
    g_odometer_motor_angle_deg = 0.0f;
    odom_last_angle_deg = 0.0f;
    odom_has_last_angle = 0;
    odom_rx_len = 0;
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void Odometer_Update(void)
{
    Emm_V5_Read_Sys_Params(ODOMETER_MOTOR_ADDR, S_CPOS);
}

void Odometer_UartRxByte(uint8_t data)
{
    if (odom_rx_len == 0)
    {
        if (data == ODOMETER_MOTOR_ADDR)
        {
            odom_rx_buf[odom_rx_len++] = data;
        }
        return;
    }

    if (odom_rx_len == 1 && data != ODOMETER_POS_CMD)
    {
        odom_rx_len = 0;
        return;
    }

    odom_rx_buf[odom_rx_len++] = data;

    if (odom_rx_len >= ODOMETER_FRAME_MAX_LEN)
    {
        if (odom_rx_buf[ODOMETER_FRAME_MAX_LEN - 1] == ODOMETER_FRAME_END)
        {
            Odometer_ParseFrame();
        }

        odom_rx_len = 0;
    }
}



float Odometer_GetDistanceCm(void)
{
    return g_odometer_distance_cm;
}

float Odometer_GetMotorAngleDeg(void)
{
    return g_odometer_motor_angle_deg;
}
