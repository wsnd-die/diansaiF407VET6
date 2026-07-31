#include "gangzhu_pid.h"
#include "Q_pid.h"
#include "UpperCP.h"
#include "bujin.h"
#include <stdbool.h>

GangzhuPid_t s_gangzhu_pid;
float step_mm = 0.0f;

/* 速度死区与平滑滤波参数 */
#define SPD_DEADBAND        15.0f   /* 速度死区阈值，绝对值小于此值视为 0 */
#define SPD_FILTER_ALPHA    0.8f    /* 一阶低通滤波系数 (0~1)，越大越平滑 */
static float s_filtered_speed = 0.0f;

static float GangzhuPid_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void GangzhuPid_Init(GangzhuPid_t *pid, float kp, float ki, float kd)
{
    const fp32 pid_params[3] = { kp, ki, kd };
    const fp32 speed_pid_params[3] = { 0.54f, 0.00f, 0.08f };

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->position_mm = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_previous_error = 0.0f;
    pid->output = 0.0f;
    pid->target_speed = 0.0f;
    pid->initialized = 0U;
    PID_init(&pid->q_pid, PID_DELTA, pid_params, 140, 10);
    PID_init(&pid->speed_pid, PID_DELTA, speed_pid_params, 140, 10);
}

void GangzhuPid_SetGains(GangzhuPid_t *pid, float kp, float ki, float kd)
{
    pid->kp = (kp > 0.0f) ? kp : 0.0f;
    pid->ki = (ki > 0.0f) ? ki : 0.0f;
    pid->kd = (kd > 0.0f) ? kd : 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_previous_error = 0.0f;
    pid->output = 0.0f;
    pid->target_speed = 0.0f;
    pid->initialized = 0U;
    pid->q_pid.Kp = pid->kp;
    pid->q_pid.Ki = pid->ki;
    pid->q_pid.Kd = pid->kd;
    PID_clear(&pid->q_pid);
    PID_clear(&pid->speed_pid);
}

void GangzhuPid_SetSpeedGains(GangzhuPid_t *pid, float kp, float ki, float kd)
{
    pid->speed_pid.Kp = (kp > 0.0f) ? kp : 0.0f;
    pid->speed_pid.Ki = (ki > 0.0f) ? ki : 0.0f;
    pid->speed_pid.Kd = (kd > 0.0f) ? kd : 0.0f;
    PID_clear(&pid->speed_pid);
    pid->output = 0.0f;
    output_gangzhu = 0.0f;
    step_mm = 0.0f;
}

void GangzhuPid_AdjustGains(GangzhuPid_t *pid, float kp_delta,
                            float ki_delta, float kd_delta)
{
    GangzhuPid_SetGains(pid, pid->kp + kp_delta,
                         pid->ki + ki_delta, pid->kd + kd_delta);
}

void GangzhuPid_AdjustSpeedGains(GangzhuPid_t *pid, float kp_delta,
                                  float ki_delta, float kd_delta)
{
    GangzhuPid_SetSpeedGains(pid, pid->speed_pid.Kp + kp_delta,
                              pid->speed_pid.Ki + ki_delta,
                              pid->speed_pid.Kd + kd_delta);
}

void Gangzhu_Control_Update(void)
{
    if ((gangzhu_err == 0) && (gangzhu_speed == 0)) {
        s_gangzhu_pid.target_speed = 0.0f;
        s_gangzhu_pid.output = 0.0f;
        output_gangzhu = 0.0f;
        step_mm = 0.0f;
        s_filtered_speed = 0.0f;
        PID_clear(&s_gangzhu_pid.q_pid);
        PID_clear(&s_gangzhu_pid.speed_pid);
        return;
    }

    if (gangzhu_err == 0) {
        s_gangzhu_pid.target_speed = 0.0f;
        PID_clear(&s_gangzhu_pid.q_pid);
    } else {
        s_gangzhu_pid.target_speed = GangzhuPid_Update(&s_gangzhu_pid, gangzhu_err);
    }

    {
        float raw_speed = (float)gangzhu_speed;

        /* 死区处理：绝对值小于阈值则视为 0 */
        if (raw_speed < SPD_DEADBAND && raw_speed > -SPD_DEADBAND) {
            raw_speed = 0.0f;
        }

        /* 一阶低通滤波：s_filtered = α·s_filtered + (1-α)·raw */
        s_filtered_speed = SPD_FILTER_ALPHA * s_filtered_speed
                         + (1.0f - SPD_FILTER_ALPHA) * raw_speed;
    }

    output_gangzhu = PID_calc(&s_gangzhu_pid.speed_pid,
                              -s_filtered_speed,
                              -s_gangzhu_pid.target_speed);
    s_gangzhu_pid.output = output_gangzhu;
     step_mm=-GangzhuPid_Clamp(output_gangzhu,-130,140);
    if (step_mm > 0.0f) {
        Emm_V5_Pos_Control_ByPulse(5, 0, 256, 177, step_mm, 1, false);
    } else if (step_mm < 0.0f) {
        Emm_V5_Pos_Control_ByPulse(5, 1, 256, 177, -step_mm, 1, false);
    }
}
 float output_gangzhu;
float GangzhuPid_Update(GangzhuPid_t *pid, short error)
{
    float error_f = (float)error;
    float output;

    output = PID_calc(&pid->q_pid, 0.0f, error_f);
    pid->previous_error = pid->q_pid.error[1];
    pid->previous_previous_error = pid->q_pid.error[2];
    pid->output = output;
    pid->initialized = 1U;

    return output;
    
}

float GangzhuPid_GetPosition(const GangzhuPid_t *pid)
{
    return pid->position_mm;
}
