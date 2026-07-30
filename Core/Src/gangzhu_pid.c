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

void GangzhuPid_Init(GangzhuPid_t *pid, float kp, float ki, float kd,
                     float period_s, float max_step_mm,
                     float min_position_mm, float max_position_mm)
{
    const fp32 pid_params[3] = { kp, ki, kd };

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->period_s = period_s;
    pid->max_step_mm = max_step_mm;
    pid->min_position_mm = min_position_mm;
    pid->max_position_mm = max_position_mm;
    pid->position_mm = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_previous_error = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0U;
    PID_init(&pid->q_pid, PID_DELTA, pid_params, 80, 10);
}

void GangzhuPid_SetGains(GangzhuPid_t *pid, float kp, float ki, float kd)
{
    pid->kp = (kp > 0.0f) ? kp : 0.0f;
    pid->ki = (ki > 0.0f) ? ki : 0.0f;
    pid->kd = (kd > 0.0f) ? kd : 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_previous_error = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0U;
    pid->q_pid.Kp = pid->kp;
    pid->q_pid.Ki = pid->ki;
    pid->q_pid.Kd = pid->kd;
    PID_clear(&pid->q_pid);
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
        return;
    }
      output_gangzhu  = GangzhuPid_Update(&s_gangzhu_pid, gangzhu_err);
     step_mm=-GangzhuPid_Clamp(output_gangzhu,-100,100);
    if (step_mm > 0.0f) {
        Emm_V5_Pos_Control(5, 0, 150, 220, step_mm, 1, false);
    } else if (step_mm < 0.0f) {
        Emm_V5_Pos_Control(5, 1, 150, 220, -step_mm, 1, false);
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
