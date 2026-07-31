/**
 * @file    fourbar.c
 * @brief   摩擦前馈补偿 —— 库伦摩擦 + 自适应学习 + 低速粘滞补偿
 *
 * @details 管壁粗糙 → 杆子需要克服静摩擦才能动 → PID 积分响应慢 → 过冲
 *          前馈直接预加克服摩擦的力，让 PID 只需处理动态误差。
 *
 *          自适应层：稳态时学习当前位置的摩擦量，坑坑洼洼自动记忆。
 *
 *          2026-08 改进：
 *          - FF_SPD_THRESHOLD 从 10 降至 3，低速也补偿
 *          - FF_FRICTION_BASE 从 5 升至 8
 *          - 新增极低速（无速度目标时）的静态偏置补偿
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
    } else {
        /* 低速/零速时：按 pid_out 方向加基础摩擦，帮助克服静摩擦 */
        if (pid_out > FF_FRICTION_BASE) {
            friction = FF_FRICTION_BASE * 0.6f;
        } else if (pid_out < -FF_FRICTION_BASE) {
            friction = -FF_FRICTION_BASE * 0.6f;
        }
        /* pid_out 太小 → 不加额外摩擦，让积分自己积累 */
    }

    /* 自适应量限幅 */
    if (s_friction_adapt > FF_MAX_ADAPTIVE) {
        s_friction_adapt = FF_MAX_ADAPTIVE;
    } else if (s_friction_adapt < -FF_MAX_ADAPTIVE) {
        s_friction_adapt = -FF_MAX_ADAPTIVE;
    }

    s_last_friction = friction;

    /* === 2. 自适应摩擦学习 === */
    {
        int32_t abs_err = (pos_error >= 0) ? pos_error : -pos_error;
        if (abs_err <= FF_STEADY_POS) {
            /* 稳态时 |pid_out| ≈ 克服摩擦所需输出 */
            s_friction_adapt = (1.0f - FF_FRICTION_LEARN) * s_friction_adapt
                             + FF_FRICTION_LEARN * fabsf(pid_out);
        }
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
