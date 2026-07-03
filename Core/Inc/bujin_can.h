#ifndef __BUJIN_CAN_H
#define __BUJIN_CAN_H

#include "main.h"
#include <stdbool.h>

/* ── CAN ID_Addr：与驱动器菜单 ID_Addr 保持一致，此处 = 步进电机地址 ── */
#define EMM_CAN_ID_ADDR  3U

/* ── CAN 接收分发（由 can.c 的 RX 回调调用） ── */
void Bujin_CAN_RxDispatch(uint32_t can_id, uint8_t *data, uint8_t len);

/* ── Emm V5 驱动器系统参数枚举（与 bujin.h 一致） ── */
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

/* ── API 与 bujin.h 完全一致，只是传输层从 USART2 换成了 CAN1 ── */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, float mm, bool raF, bool snF);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc);

#endif
