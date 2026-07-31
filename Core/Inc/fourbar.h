/**
 * @file    fourbar.h
 * @brief   摩擦前馈补偿模块
 */

#ifndef __FOURBAR_H
#define __FOURBAR_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 *   前馈补偿参数（可在线调）
 * ====================================================================== */
#define FF_FRICTION_BASE     8.0f    /**< 基础库伦摩擦（脉冲）, 原5.0偏低 */
#define FF_FRICTION_LEARN    0.05f   /**< 自适应摩擦学习率, 原0.03偏慢 */
#define FF_SPD_THRESHOLD     3.0f    /**< 摩擦前馈启用的最低 speed 阈值, 原10过高导致低速不补偿 */
#define FF_STEADY_POS        3       /**< 自适应学习所需的稳态位置阈值 */
#define FF_MAX_ADAPTIVE      40.0f   /**< 自适应摩擦上限（防跑飞）, 原30偏低 */

/* ======================================================================
 *   API
 * ====================================================================== */

void FF_Init(void);
void FF_Reset(void);

/**
 * @brief  摩擦前馈补偿
 * @param  pid_out       原始 PID 叠加输出
 * @param  target_speed  速度环目标（>0 正方向, <0 反方向）
 * @param  pos_error     位置误差（用于自适应学习）
 * @return 补偿后的输出 (= pid_out + friction)
 */
float FF_Compensate(float pid_out, float target_speed, int16_t pos_error);

float FF_GetFriction(void);
float FF_GetAdaptiveFriction(void);

#endif /* __FOURBAR_H */
