#ifndef GANGZHU_PID_H
#define GANGZHU_PID_H

#include "Q_pid.h"
#include <stdbool.h>

typedef struct {
    pid_type_def q_pid;      /* 距离外环 PID 控制器 */
    pid_type_def speed_pid;  /* 速度内环 PID 控制器 */
    float kp;                /* 比例系数 */
    float ki;                /* 积分系数 */
    float kd;                /* 微分系数 */
    float position_mm;       /* 当前软件位置，单位 mm */
    float previous_error;    /* 上次误差 */
    float previous_previous_error; /* 上上次误差 */
    float output;            /* 上次 PID 输出 */
    float target_speed;      /* 速度环目标速度 */
    bool speed_enabled;      /* 速度环使能标志 */
    bool outer_enabled;
    float pos_out;
    float spd_out;
    unsigned char initialized;
} GangzhuPid_t;
extern GangzhuPid_t s_gangzhu_pid;
extern volatile float step_mm;
extern volatile float output_gangzhu;
void Gangzhu_Control_Update(void);
void GangzhuPid_Init(GangzhuPid_t *pid, float kp, float ki, float kd);
void GangzhuPid_SetGains(GangzhuPid_t *pid, float kp, float ki, float kd);
void GangzhuPid_SetSpeedGains(GangzhuPid_t *pid, float kp, float ki, float kd);
void GangzhuPid_AdjustGains(GangzhuPid_t *pid, float kp_delta,
                            float ki_delta, float kd_delta);
void GangzhuPid_AdjustSpeedGains(GangzhuPid_t *pid, float kp_delta,
                                  float ki_delta, float kd_delta);
void GangzhuPid_ResetState(GangzhuPid_t *pid);
void GangzhuPid_SetOuterEnabled(GangzhuPid_t *pid, bool enabled);
float GangzhuPid_GetFilteredSpeed(void);
float GangzhuPid_Update(GangzhuPid_t *pid, short error);
float GangzhuPid_GetPosition(const GangzhuPid_t *pid);

#endif
