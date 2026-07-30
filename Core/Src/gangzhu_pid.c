#include "gangzhu_pid.h"
#include "Q_pid.h"
#include "UpperCP.h"
#include "bujin.h"
#include <stdbool.h>

GangzhuPid_t s_gangzhu_pid;
float step_mm = 0.0f;
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
    const fp32 speed_pid_params[3] = { 0.48f, 0.002f, 0.002f };

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
    PID_init(&pid->speed_pid, PID_DELTA, speed_pid_params, 100, 10);
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

void GangzhuPid_AdjustGains(GangzhuPid_t *pid, float kp_delta,
                            float ki_delta, float kd_delta)
{
    GangzhuPid_SetGains(pid, pid->kp + kp_delta,
                         pid->ki + ki_delta, pid->kd + kd_delta);
}
void Gangzhu_Control_Update(void)
{
    if (gangzhu_err == 0) {
        s_gangzhu_pid.target_speed = 0.0f;
        s_gangzhu_pid.output = 0.0f;
        output_gangzhu = 0.0f;
        step_mm = 0.0f;
        PID_clear(&s_gangzhu_pid.q_pid);
        PID_clear(&s_gangzhu_pid.speed_pid);
        return;
    }

    s_gangzhu_pid.target_speed = GangzhuPid_Update(&s_gangzhu_pid, gangzhu_err);
    output_gangzhu = PID_calc(&s_gangzhu_pid.speed_pid,
                              -(float)gangzhu_speed,
                              s_gangzhu_pid.target_speed);
    s_gangzhu_pid.output = output_gangzhu;
     step_mm=-GangzhuPid_Clamp(output_gangzhu,-100,100);
    if (step_mm > 0.0f) {
        Emm_V5_Pos_Control_ByPulse(5, 0, 150, 220, step_mm, 1, false);
    } else if (step_mm < 0.0f) {
        Emm_V5_Pos_Control_ByPulse(5, 1, 150, 220, -step_mm, 1, false);
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
