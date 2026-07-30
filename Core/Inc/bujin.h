#ifndef __BUJIN_H
#define __BUJIN_H

#include "main.h"
#include <stdbool.h>

typedef enum {
    S_VER   = 0,
    S_RL    = 1,
    S_PID   = 2,
    S_VBUS  = 3,
    S_CPHA  = 5,
    S_ENCL  = 7,
    S_TPOS  = 8,
    S_VEL   = 9,
    S_CPOS  = 10,
    S_PERR  = 11,
    S_FLAG  = 13,
    S_Conf  = 14,
    S_State = 15,
    S_ORG   = 16,
} SysParams_t;

typedef enum {
    EMM_V5_ZERO_SINGLE_NEAREST = 0, /* 单圈就近回零 */
    EMM_V5_ZERO_SINGLE_DIR     = 1, /* 单圈方向回零 */
    EMM_V5_ZERO_MULTI_COLLISION = 2, /* 多圈无限位碰撞回零 */
    EMM_V5_ZERO_MULTI_LIMIT    = 3, /* 多圈有限位开关回零 */
} Emm_V5_Zero_Mode_t;

typedef struct {
    Emm_V5_Zero_Mode_t mode; /* 回零模式 */
    uint8_t direction;        /* 回零方向：0 为 CW，1 为 CCW */
    uint16_t speed_rpm;       /* 回零转速，单位 RPM */
    uint32_t timeout_ms;      /* 回零超时时间，单位 ms */
    uint16_t collision_rpm;   /* 碰撞回零检测转速，单位 RPM */
    uint16_t collision_ma;    /* 碰撞回零检测电流，单位 mA */
    uint16_t collision_ms;    /* 碰撞回零检测时间，单位 ms */
    bool auto_trigger;        /* 上电后是否自动触发回零 */
} Emm_V5_Zero_Params_t;

#define EMM_V5_ZERO_STATUS_ENCODER_READY  (1U << 0)
#define EMM_V5_ZERO_STATUS_TABLE_READY    (1U << 1)
#define EMM_V5_ZERO_STATUS_RUNNING        (1U << 2)
#define EMM_V5_ZERO_STATUS_FAILED         (1U << 3)

void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Set_Zero(uint8_t addr, bool save);
void Emm_V5_Trigger_Zero(uint8_t addr, Emm_V5_Zero_Mode_t mode, bool snF);
void Emm_V5_Read_Zero_Params(uint8_t addr);
void Emm_V5_Modify_Zero_Params(uint8_t addr, bool save, const Emm_V5_Zero_Params_t *params);
void Emm_V5_Read_Zero_Status(uint8_t addr);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc);
void Emm_V5_Pos_Control_ByPulse(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulse, bool raF, bool snF);
#endif
