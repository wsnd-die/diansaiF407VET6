/**
 * @file    fourbar.c
 * @brief   摩擦前馈补偿 —— 库伦摩擦 + 自适应学习
 *
 * @details 管壁粗糙 → 杆子需要克服静摩擦才能动 → PID 积分响应慢 → 过冲
 *          前馈直接预加克服摩擦的力，让 PID 只需处理动态误差。
 *
 *          自适应层：稳态时学习当前位置的摩擦量，坑坑洼洼自动记忆。
 */

#include "fourbar.h"
#include <math.h>

/* ======================================================================
 *   内部状态
 * ====================================================================== */

static float s_friction_adapt = 0.0f;   /* 自适应摩擦估计 */
static float s_last_friction  = 0.0f;   /* 上拍实际输出的摩擦值 */
static bool  s_ready          = false;

/* ======================================================================
 *   API
 * ====================================================================== */

void FF_Init(void)
{
    s_friction_adapt = 0.0f;
    s_last_friction  = 0.0f;
    s_ready          = true;
}

void FF_Reset(void)
{
    s_friction_adapt = 0.0f;
    s_last_friction  = 0.0f;
}

float FF_Compensate(float pid_out, float target_speed, int16_t pos_error)
{
    float friction = 0.0f;

    if (!s_ready) {
        return pid_out;
    }

    /* === 1. 库伦摩擦前馈 === */
    if (target_speed > FF_SPD_THRESHOLD) {
        friction = FF_FRICTION_BASE + fabsf(s_friction_adapt);
    } else if (target_speed < -FF_SPD_THRESHOLD) {
        friction = -(FF_FRICTION_BASE + fabsf(s_friction_adapt));
    }
    /* target_speed 在 ±阈值之间 → 不加摩擦，让积分自己稳住 */

    /* 自适应量限幅 */
    if (s_friction_adapt > FF_MAX_ADAPTIVE) {
        s_friction_adapt = FF_MAX_ADAPTIVE;
    }

    s_last_friction = friction;

    /* === 2. 自适应摩擦学习 === */
    if ((pos_error >= -FF_STEADY_POS) && (pos_error <= FF_STEADY_POS)) {
        /* 稳态时 |pid_out| ≈ 克服摩擦所需输出 */
        s_friction_adapt = (1.0f - FF_FRICTION_LEARN) * s_friction_adapt
                         + FF_FRICTION_LEARN * fabsf(pid_out);
    }

    return pid_out + friction;
}

float FF_GetFriction(void)
{
    return s_last_friction;
}

float FF_GetAdaptiveFriction(void)
{
    return s_friction_adapt;
}
