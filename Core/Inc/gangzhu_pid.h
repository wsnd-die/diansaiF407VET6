#ifndef GANGZHU_PID_H
#define GANGZHU_PID_H

#include "Q_pid.h"

typedef struct {
    pid_type_def q_pid;      /* 通用增量式 PID 控制器 */
    float kp;                /* 比例系数 */
    float ki;                /* 积分系数 */
    float kd;                /* 微分系数 */
    float position_mm;       /* 当前软件位置，单位 mm */
    float previous_error;    /* 上次误差 */
    float previous_previous_error; /* 上上次误差 */
    float output;            /* 上次 PID 输出 */
    unsigned char initialized;
} GangzhuPid_t;
extern GangzhuPid_t s_gangzhu_pid;
extern float step_mm;
extern float output_gangzhu;
void Gangzhu_Control_Update(void);
void GangzhuPid_Init(GangzhuPid_t *pid, float kp, float ki, float kd);
void GangzhuPid_SetGains(GangzhuPid_t *pid, float kp, float ki, float kd);
void GangzhuPid_AdjustGains(GangzhuPid_t *pid, float kp_delta,
                            float ki_delta, float kd_delta);
float GangzhuPid_Update(GangzhuPid_t *pid, short error);
float GangzhuPid_GetPosition(const GangzhuPid_t *pid);

#endif
