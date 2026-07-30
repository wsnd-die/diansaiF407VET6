/**
 * @file    fourbar.h
 * @brief   四连杆机构运动学解算模块
 *
 * @details 机构描述：
 *          步进电机驱动曲柄 (r)，通过连杆 (L) 带动摆杆 (R)。
 *          - 固定铰链 A = (0, 0)
 *          - 步进电机轴心 O = (Ox, Oy) = (208, -21.24)
 *          - 曲柄 OB 长 r = 45mm，由步进电机驱动
 *          - 摆杆 AC 长 R = 247mm，绕固定铰链 (0,0) 摆动
 *          - 连杆 BC 长 L = 21.24mm，连接曲柄末端与摆杆末端
 *
 *                C (摆杆末端) -- L (连杆) -- B (曲柄末端)
 *                |                            |
 *            R (摆杆)                      r (曲柄)
 *                |                            |
 *                A (固定铰链)              O (步进电机)
 *                (0, 0)                    (208, -21.24)
 *
 *          输入:  摆杆倾角 phi_deg（角度制，°），以 x 轴正向为 0°
 *          输出:  步进电机转角 theta_deg（角度制，°），
 *                 以及对应的步进脉冲数（用于 "张大头42步进电机"）。
 *
 *          解法:  已知 C 点坐标 → 圆 O(r) 与圆 C(L) 求交 → B 点 → θ。
 */

#ifndef __FOURBAR_H
#define __FOURBAR_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 *   机械常数（单位：mm）
 * ====================================================================== */
#define FOURBAR_R_MM        247.0f    /**< 摆杆长度 */
#define FOURBAR_OX_MM       208.0f    /**< 步进电机轴心 O 的 X 坐标 */
#define FOURBAR_OY_MM       (-21.24f) /**< 步进电机轴心 O 的 Y 坐标 */
#define FOURBAR_r_MM        45.0f     /**< 曲柄长度 */
#define FOURBAR_L_MM        21.24f    /**< 连杆长度 */

/* ======================================================================
 *   步进电机参数（张大头 42 步进电机，可调）
 * ====================================================================== */
#define FOURBAR_STEPS_PER_REV   (3200U) /**< 单圈脉冲数（16 细分时 200×16=3200） */

/* ======================================================================
 *   解算结果结构体
 * ====================================================================== */
typedef struct {
    float theta1_deg;        /**< 电机转角方案 1，° */
    float theta2_deg;        /**< 电机转角方案 2，° */
    float pulley_deg;        /**< 实际选用的电机转角，° */
    float rocker_phi_deg;    /**< 当前输入的摆杆倾角，° */
    int32_t pulses;          /**< 本次相对上一拍的脉冲增量 */
    int8_t  solution_index;  /**< 选用的解：1 或 2；无解则为 0 */
    bool    valid;           /**< 解算是否成功（机构可装配 = true） */
} FourBar_Result_t;

/* ======================================================================
 *   公开 API
 * ====================================================================== */

/**
 * @brief   初始化解算模块（预计算机架参数与上拍记忆）
 */
void FourBar_Init(void);

/**
 * @brief   根据摆杆倾角解算步进电机转角与脉冲增量
 *
 * @param   phi_deg         摆杆倾角（°），以 x 轴正向为 0，逆时针为正
 * @param   current_pulses  步进电机当前绝对脉冲计数值（保留参数）
 * @return  FourBar_Result_t 解算结果。若 valid == false 则 phi 超出机构运动范围
 */
FourBar_Result_t FourBar_Solve(float phi_deg, int32_t current_pulses);

/**
 * @brief   电机转角 → 脉冲计数值（单圈内绝对位置）
 */
int32_t FourBar_AngleToPulses(float angle_deg);

/**
 * @brief   脉冲计数值 → 电机转角
 */
float FourBar_PulsesToAngle(int32_t pulses);

/**
 * @brief   正向运动学：电机转角 theta → 摆杆倾角 phi（回代验证用）
 * @return  phi（°）；超出范围返回 NaN
 */
float FourBar_Forward(float theta_deg);

#endif /* __FOURBAR_H */
