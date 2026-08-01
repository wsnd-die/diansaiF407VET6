#include "gangzhu_pid.h"
#include "Q_pid.h"
#include "UpperCP.h"
#include "bujin.h"
#include <stdbool.h>

GangzhuPid_t s_gangzhu_pid;
volatile float step_mm = 0.0f;

/* 速度死区与平滑滤波参数 */
#define SPD_DEADBAND        15.0f   /* 速度死区阈值，绝对值小于此值视为 0 */
#define SPD_FILTER_ALPHA    0.3f    /* 一阶低通滤波系数 (0~1)，越大越平滑 */
static float s_filtered_speed = 0.0f;

/* 输出平滑：一阶低通 + slew-rate 限制，允许高加速度不抖 */
#define OUTPUT_LPF_ALPHA        0.25f  /* 输出低通 (越小响应越快) */
#define OUTPUT_SLEW_MAX         200.0f  /* 每拍最大变化量，防突变 */
static float s_filtered_output = 0.0f;
static bool  s_output_inited = false;
float volatile acc_cmp = 0.0f;

/* 加速度前馈默认参数 (仿 PC 端 feedforward) */
#define ACC_FF_THRESHOLD        0.1f    /* 阈值 (g): |acc|<此值不启用前馈 */
#define ACC_FF_TIMEOUT_TICKS    400     /* 触发后 4s 强制归零 */
#define ACC_FF_DEFAULT_GAIN     2000.0f  /* 默认增益 (mm/g) */
#define ACC_FF_DEFAULT_MAX      180.0f   /* 默认限幅 (mm) */
#define ACC_FF_STALE_TICKS      20      /* 数据过期拍数, 超时归零 */

/* 电机指令限频 */
#define MOTOR_CMD_INTERVAL      2.5       /* 电机指令间隔 (拍数) */
#define MOTOR_CMD_DEADBAND      3.0f    /* 变化小于此值不重发 (mm) */

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
    const fp32 speed_pid_params[3] = { 0.5f, 0.001f, 0.39f };
//	const fp32 speed_pid_params[3] = { 0.0f, 0.00f, 0.0f };

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->position_mm = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_previous_error = 0.0f;
    pid->output = 0.0f;
    pid->target_speed = 0.0f;
    pid->initialized = 0U;
    pid->car_acc_gain = ACC_FF_DEFAULT_GAIN;
    pid->car_acc_max  = ACC_FF_DEFAULT_MAX;
    pid->car_acc_feedforward_mm = 0.0f;
    pid->car_acc_fresh = false;
    PID_init(&pid->q_pid, PID_POSITION, pid_params, 170, 10);
    PID_init(&pid->speed_pid, PID_POSITION, speed_pid_params, 170, 10);
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
    static uint32_t s_motor_tick = 0;
    static float    s_motor_last = 0.0f;
    static int      s_ff_stale   = 0;    /* 前馈过期计数器 */

    if ((gangzhu_err == 0) && (gangzhu_speed == 0)) {
        s_gangzhu_pid.target_speed = 0.0f;
        s_gangzhu_pid.output = 0.0f;
        output_gangzhu = 0.0f;
        step_mm = 0.0f;
        s_filtered_speed = 0.0f;
        s_filtered_output = 0.0f;
        s_output_inited = false;
        acc_cmp = 0.0f;
        s_motor_tick = 0;
        s_motor_last = 0.0f;
        s_ff_stale = 0;
        s_gangzhu_pid.car_acc_feedforward_mm = 0.0f;
        s_gangzhu_pid.car_acc_fresh = false;
        PID_clear(&s_gangzhu_pid.q_pid);
        PID_clear(&s_gangzhu_pid.speed_pid);
        return;
    }

    if (gangzhu_err == 0) {
        s_gangzhu_pid.target_speed = 0.0f;
        PID_clear(&s_gangzhu_pid.q_pid);
    }
    s_gangzhu_pid.target_speed = GangzhuPid_Update(&s_gangzhu_pid, gangzhu_err);

    /* 速度滤波 */
    {
        float raw_speed = (float)gangzhu_speed;

        if (raw_speed < SPD_DEADBAND && raw_speed > -SPD_DEADBAND) {
            raw_speed = 0.0f;
        }

        s_filtered_speed = SPD_FILTER_ALPHA * s_filtered_speed
                         + (1.0f - SPD_FILTER_ALPHA) * raw_speed;
    }

    /* 速度环 PID */
    output_gangzhu = PID_calc(&s_gangzhu_pid.speed_pid,
                              -s_filtered_speed,
                              s_gangzhu_pid.target_speed);
    s_gangzhu_pid.output = output_gangzhu;
    step_mm = -GangzhuPid_Clamp(output_gangzhu, -160.0f, 170.0f);

    /* ==== 加速度前馈: 静态偏置 (外部 setter → 内部消费, 仿 PC feedforward) ==== */
    if (s_gangzhu_pid.car_acc_gain != 0.0f) {
        /* 过期保护: 超时未刷新 → 前馈归零 */
        if (s_gangzhu_pid.car_acc_fresh) {
            s_ff_stale = 0;
        } else {
            s_ff_stale++;
        }

        if (s_ff_stale > ACC_FF_STALE_TICKS) {
            s_gangzhu_pid.car_acc_feedforward_mm = 0.0f;
        }

        acc_cmp = s_gangzhu_pid.car_acc_feedforward_mm;
        step_mm -= acc_cmp;
    }

    /* ==== 输出一阶低通 + slew-rate 限制 ==== */
    {
        float raw = step_mm;

        if (!s_output_inited) {
            s_filtered_output = raw;
            s_output_inited = true;
        } else {
            float filtered = OUTPUT_LPF_ALPHA * s_filtered_output
                           + (1.0f - OUTPUT_LPF_ALPHA) * raw;
            float delta = filtered - s_filtered_output;
            if (delta > OUTPUT_SLEW_MAX) {
                filtered = s_filtered_output + OUTPUT_SLEW_MAX;
            } else if (delta < -OUTPUT_SLEW_MAX) {
                filtered = s_filtered_output - OUTPUT_SLEW_MAX;
            }
            s_filtered_output = filtered;
        }

        s_motor_tick++;

        float change = s_filtered_output - s_motor_last;
        if (change < 0.0f) change = -change;

        if (s_motor_tick >= MOTOR_CMD_INTERVAL || change >= MOTOR_CMD_DEADBAND) {
            s_motor_tick = 0;
            s_motor_last = s_filtered_output;

            if (s_filtered_output > 0.0f) {
                Emm_V5_Pos_Control_ByPulse(5, 0, 800, 230, (uint32_t)(s_filtered_output + 0.5f), 1, false);
            } else if (s_filtered_output < 0.0f) {
                Emm_V5_Pos_Control_ByPulse(5, 1, 800, 230, (uint32_t)(-s_filtered_output + 0.5f), 1, false);
            }
        }
    }
}
 volatile float output_gangzhu;
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
    if (pid == NULL) return;
    pid->output = 0.0f;
    pid->target_speed = 0.0f;
    output_gangzhu = 0.0f;
    step_mm = 0.0f;
    s_filtered_speed = 0.0f;
    s_filtered_output = 0.0f;
    s_output_inited = false;
    acc_cmp = 0.0f;
    pid->car_acc_feedforward_mm = 0.0f;
    pid->car_acc_fresh = false;
    PID_clear(&pid->q_pid);
    PID_clear(&pid->speed_pid);
}

void GangzhuPid_SetOuterEnabled(GangzhuPid_t *pid, bool enabled)
{
    if (pid == NULL) return;
    pid->outer_enabled = enabled;
    if (!enabled) {
        pid->target_speed = 0.0f;
        PID_clear(&pid->q_pid);
    }
}

/* ========== 加速度前馈 API (外部调用, 仿 PC set_car_acceleration_feedforward) ========== */

/**
 * @brief 外部传入加速度前馈值
 * @param acc_y_g  滤波后的 Y 轴加速度 (单位 g)
 * @note  由 Task02 每 10ms 调用一次
 */
void GangzhuPid_SetAccFeedforward(float acc_y_g)
{
    static int   s_ff_timer = 0;       /* 触发后计时器 (拍, 100Hz) */
    static bool  s_ff_active = false;   /* 是否处于前馈窗口内 */
    static bool  s_ff_done = false;     /* 一次性触发标志: true=已触发过,不再响应 */

    float abs_acc = (acc_y_g > 0.0f) ? acc_y_g : -acc_y_g;

    /* 一次性触发: 首次 |acc| > 阈值 → 开启, 之后永不触发 */
    if (!s_ff_done && abs_acc > ACC_FF_THRESHOLD) {
        s_ff_done = true;
        s_ff_active = true;
        s_ff_timer = 0;
    }

    /* 计时: 4s 后关闭 */
    if (s_ff_active) {
        s_ff_timer++;
        if (s_ff_timer >= ACC_FF_TIMEOUT_TICKS) {
            s_ff_active = false;
        }
    }

    /* 输出: 窗口内 = acc×增益, 窗口外 = 0 */
    if (s_ff_active) {
        float ff = acc_y_g * s_gangzhu_pid.car_acc_gain;
        if (ff > s_gangzhu_pid.car_acc_max)  ff = s_gangzhu_pid.car_acc_max;
        if (ff < -s_gangzhu_pid.car_acc_max) ff = -s_gangzhu_pid.car_acc_max;
        s_gangzhu_pid.car_acc_feedforward_mm = ff;
    } else {
        s_gangzhu_pid.car_acc_feedforward_mm = 0.0f;
    }

    s_gangzhu_pid.car_acc_fresh = true;
}

float GangzhuPid_GetAccFeedforward(void)
{
    return s_gangzhu_pid.car_acc_feedforward_mm;
}

void GangzhuPid_SetAccGain(float gain)
{
    s_gangzhu_pid.car_acc_gain = gain;
}

float GangzhuPid_GetAccGain(void)
{
    return s_gangzhu_pid.car_acc_gain;
}
