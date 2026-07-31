#include "gangzhu_pid.h"
#include "Q_pid.h"
#include "UpperCP.h"
#include "bujin.h"
#include "fourbar.h"
#include <stdbool.h>

GangzhuPid_t s_gangzhu_pid;
volatile float step_mm = 0.0f;
volatile float output_gangzhu = 0.0f;

/* �е�ͣ���������ٶ�ƽ���˲����� */
#define POS_DEADBAND        3
#define SPD_DEADBAND        10.0f
#define SPD_FILTER_ALPHA    0.8f
static float s_filtered_speed = 0.0f;
#define OUTPUT_LPF_ALPHA    0.7f
static float s_filtered_output = 0.0f;
static bool s_first_filter_run = true;

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
    const fp32 speed_pid_params[3] = { 0.55f, 0.00f, 0.10f };
	//const fp32 speed_pid_params[3] = { 0.59f, 0.001f, 0.1f };

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->position_mm = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_previous_error = 0.0f;
    pid->output = 0.0f;
    pid->target_speed = 0.0f;
    pid->speed_enabled = 1;
    pid->initialized = 0U;
    PID_init(&pid->q_pid, PID_POSITION, pid_params, 140, 10);
    PID_init(&pid->speed_pid, PID_POSITION, speed_pid_params, 140, 10);

    FF_Init();
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
           s_gangzhu_pid.pos_out = 0.0f;
            s_gangzhu_pid.spd_out = 0.0f;

    /* λ�����ٶ�ͬʱ��������ʱͣ���������������ʷ״̬�� */
    if ((gangzhu_err >= -POS_DEADBAND) &&
        (gangzhu_err <= POS_DEADBAND) &&
        ((float)gangzhu_speed >= -SPD_DEADBAND) &&
        ((float)gangzhu_speed <= SPD_DEADBAND)) {
        s_gangzhu_pid.output = 0.0f;
        output_gangzhu = 0.0f;
        step_mm = 0.0f;
        s_filtered_speed = 0.0f;
        PID_clear(&s_gangzhu_pid.q_pid);
        PID_clear(&s_gangzhu_pid.speed_pid);
        return;
    }

    /* ========== λ�û���error ?? PID ?? pos_out ========== */
    if (gangzhu_err != 0) {
        s_gangzhu_pid.pos_out = GangzhuPid_Update(&s_gangzhu_pid, gangzhu_err);
    } else {
        PID_clear(&s_gangzhu_pid.q_pid);
    }

    /* ========== �ٶȷ�������?? + ��ͨ��?? ========== */
    {
        float raw_speed = (float)gangzhu_speed;

        if (raw_speed < SPD_DEADBAND && raw_speed > -SPD_DEADBAND) {
            raw_speed = 0.0f;
        }

        s_filtered_speed = SPD_FILTER_ALPHA * s_filtered_speed
                         + (1.0f - SPD_FILTER_ALPHA) * raw_speed;
    }

    /* ========== �ٶȻ������� PID����ʹ��ʱ�����?? ========== */
    if (s_gangzhu_pid.speed_enabled) {
        s_gangzhu_pid.spd_out = PID_calc(&s_gangzhu_pid.speed_pid,
                           -s_filtered_speed,
                           s_gangzhu_pid.target_speed);
    } else {
        PID_clear(&s_gangzhu_pid.speed_pid);
    }

    /* ========== ������� ========== */
    output_gangzhu = s_gangzhu_pid.pos_out + s_gangzhu_pid.spd_out;
    {
        float compensated = FF_Compensate(output_gangzhu,
                                           s_gangzhu_pid.target_speed,
                                           gangzhu_err);
        output_gangzhu = compensated;
    }
    s_gangzhu_pid.output = output_gangzhu;
    step_mm = GangzhuPid_Clamp(output_gangzhu, -130.0f, 130.0f);

//    if (step_mm > 0.0f) {
//        Emm_V5_Pos_Control_ByPulse(5, 1, 500, 240,
//                                   (uint32_t)step_mm, 1, false);
//    } else if (step_mm < 0.0f) {
//        Emm_V5_Pos_Control_ByPulse(5, 0, 500, 240,
//                                   (uint32_t)(-step_mm), 1, false);
//    }
    if (s_first_filter_run) {
        s_filtered_output = step_mm;
        s_first_filter_run = false;
    } else {
        s_filtered_output = OUTPUT_LPF_ALPHA * s_filtered_output + (1.0f - OUTPUT_LPF_ALPHA) * step_mm;
    }

    // 取整并驱动
    uint32_t pulse_abs;
    uint8_t dir;
    if (s_filtered_output > 0.0f) {
        dir = 1;
        pulse_abs = (uint32_t)(s_filtered_output + 0.5f);
    } else if (s_filtered_output < 0.0f) {
        dir = 0;
        pulse_abs = (uint32_t)(-s_filtered_output + 0.5f);
    } else {
        dir = 1;
        pulse_abs = 0;
    }

    if (pulse_abs > 0) {
        Emm_V5_Pos_Control_ByPulse(5, dir, 700, 240, pulse_abs, 1, false);
    } else {
        // 可选：停止电机
        // Emm_V5_Pos_Control_ByPulse(5, dir, 500, 250, 0, 1, false);
    }
}

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

void GangzhuPid_ResetState(GangzhuPid_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->previous_error = 0.0f;
    pid->previous_previous_error = 0.0f;
    pid->output = 0.0f;
    pid->target_speed = 0.0f;
    pid->initialized = 0U;
    s_filtered_speed = 0.0f;
    output_gangzhu = 0.0f;
    step_mm = 0.0f;
    PID_clear(&pid->q_pid);
    PID_clear(&pid->speed_pid);
    
	s_filtered_output = 0.0f;
	s_first_filter_run = true; 	
    FF_Reset();
}

void GangzhuPid_SetOuterEnabled(GangzhuPid_t *pid, bool enabled)
{
    if (pid == NULL) {
        return;
    }
    pid->speed_enabled = enabled;
    if (!enabled) {
        pid->target_speed = 0.0f;
        PID_clear(&pid->q_pid);
    }
}

float GangzhuPid_GetFilteredSpeed(void)
{
    return s_filtered_speed;
}
