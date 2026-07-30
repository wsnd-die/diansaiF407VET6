#ifndef GANGZHU_PID_H
#define GANGZHU_PID_H

#include "Q_pid.h"

typedef struct {
    pid_type_def q_pid;      /* 通用增量式 PID 控制器 */
    float kp;                /* 比例系数 */
    float ki;                /* 积分系数 */
    float kd;                /* 微分系数 */
    float period_s;          /* 控制周期，单位 s */
    float max_step_mm;       /* 单次最大位移，单位 mm */
    float min_position_mm;   /* 最小机械位置，单位 mm */
    float max_position_mm;   /* 最大机械位置，单位 mm */
    float position_mm;       /* 当前软件位置，单位 mm */
    float previous_error;    /* 上次误差 */
    float previous_previous_error; /* 上上次误差 */
    float output;            /* 上次 PID 输出 */
    unsigned char initialized;
} GangzhuPid_t;
#define GANGZHU_PID_PERIOD_S       0.01f    // PID控制/采样周期 (0.01秒 = 10毫秒)
#define GANGZHU_PID_MAX_STEP_MM    8.0f   // 单次控制最大允许步距/输出限制 (单位: mm)
#define GANGZHU_POSITION_MIN_MM   -100.0f   // 机械软件限位 - 最小下限位置 (单位: mm)
#define GANGZHU_POSITION_MAX_MM    150.0f   // 机械软件限位 - 最大上限位置 (单位: mm)
extern GangzhuPid_t s_gangzhu_pid;
extern float step_mm;
extern float output_gangzhu;
void Gangzhu_Control_Update(void);
void GangzhuPid_Init(GangzhuPid_t *pid, float kp, float ki, float kd,
                     float period_s, float max_step_mm,
                     float min_position_mm, float max_position_mm);
void GangzhuPid_SetGains(GangzhuPid_t *pid, float kp, float ki, float kd);
void GangzhuPid_AdjustGains(GangzhuPid_t *pid, float kp_delta,
                            float ki_delta, float kd_delta);
float GangzhuPid_Update(GangzhuPid_t *pid, short error);
float GangzhuPid_GetPosition(const GangzhuPid_t *pid);

#endif
