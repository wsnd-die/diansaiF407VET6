/**
 * @file    fourbar.c
 * @brief   四连杆机构运动学解算实现
 *
 * @details 坐标系（世界坐标）:
 *              A = (0, 0)                   固定铰链（摆杆旋转中心）
 *              O = (Ox, Oy) = (208, -21.24) 步进电机轴心
 *
 *          在 O 为中心的局部坐标系（O' = (0,0)）中:
 *              A' = A - O = (-Ox, -Oy)
 *              C' = A' + R·(cosφ, sinφ)  -- 摆杆末端
 *              B' = r·(cosθ, sinθ)        -- 曲柄末端
 *              |C' - B'| = L              -- 连杆长度约束
 *
 *          逆解 (φ → θ): C'已知 → 圆(O',r) ∩ 圆(C',L) → B' → θ
 *          正解 (θ → φ): B'已知 → 圆(A',R) ∩ 圆(B',L) → C' → φ
 */

#include "fourbar.h"
#include <math.h>

/* ======================================================================
 *   内部状态
 * ====================================================================== */
static float s_last_theta_deg = 0.0f;   /* 上拍选用的电机转角 */
static bool  s_initialized     = false;

/* ======================================================================
 *   常量
 * ====================================================================== */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD2DEG(r)  ((r) * 180.0f / (float)M_PI)
#define DEG2RAD(d)  ((d) * (float)M_PI / 180.0f)
#define SQ(x)       ((x) * (x))

/* ======================================================================
 *   内部函数
 * ====================================================================== */

/**
 * @brief   角度规范化到 [0, 360)
 */
static float norm_360(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

/**
 * @brief   两角最短带符号差，∈ (-180, 180]
 */
static float angle_delta(float a, float b)
{
    float d = norm_360(a) - norm_360(b);
    if (d > 180.0f)    d -= 360.0f;
    if (d <= -180.0f)  d += 360.0f;
    return d;
}

/* ======================================================================
 *   公开 API
 * ====================================================================== */

void FourBar_Init(void)
{
    s_last_theta_deg = 0.0f;
    s_initialized    = true;
}

/**
 * @brief   逆解：摆杆倾角 φ → 电机转角 θ
 *
 * @details 步骤:
 *          1. C' = (R·cosφ - Ox,  R·sinφ - Oy)
 *          2. K  = (r² + |C'|² - L²) / (2·r)
 *          3. α  = atan2(C'y, C'x),  D = |C'|
 *          4. θ  = α ± acos(K/D)
 *          5. 两个解中取与上拍 θ 更接近的那个
 */
FourBar_Result_t FourBar_Solve(float phi_deg, int32_t current_pulses)
{
    FourBar_Result_t result = {0};
    float phi_rad, Cpx, Cpy, D2, D_val, K, alpha, cos_delta, delta;
    float theta1, theta2, prev;
    int32_t abs_target;
    (void)current_pulses;

    if (!s_initialized) {
        FourBar_Init();
    }

    prev = s_last_theta_deg;

    /* 1. 摆杆末端 C 在世界坐标：A=(0,0)，C = R·(cosφ, sinφ) */
    phi_rad = DEG2RAD(phi_deg);
    float Cx_w = FOURBAR_R_MM * cosf(phi_rad);
    float Cy_w = FOURBAR_R_MM * sinf(phi_rad);

    /* 2. 转到电机中心坐标系 O' = (0,0)：C' = C - O */
    Cpx = Cx_w - FOURBAR_OX_MM;
    Cpy = Cy_w - FOURBAR_OY_MM;

    /* 3. 计算辅助量 */
    D2   = SQ(Cpx) + SQ(Cpy);
    D_val = sqrtf(D2);
    K    = (SQ(FOURBAR_r_MM) + D2 - SQ(FOURBAR_L_MM))
           / (2.0f * FOURBAR_r_MM);

    /* 4. 判断机构可装配性 */
    cos_delta = K / D_val;

    if (cos_delta > 1.0f || cos_delta < -1.0f) {
        result.valid          = false;
        result.solution_index = 0;
        result.pulley_deg     = prev;
        result.theta1_deg     = 0.0f;
        result.theta2_deg     = 0.0f;
        result.rocker_phi_deg = phi_deg;
        result.pulses         = 0;
        return result;
    }

    /* 5. 两解 */
    alpha = atan2f(Cpy, Cpx);
    delta = acosf(cos_delta);

    theta1 = norm_360(RAD2DEG(alpha + delta));
    theta2 = norm_360(RAD2DEG(alpha - delta));

    result.theta1_deg = theta1;
    result.theta2_deg = theta2;
    result.valid      = true;

    /* 6. 选与上拍更接近的解 */
    float d1 = fabsf(angle_delta(theta1, prev));
    float d2 = fabsf(angle_delta(theta2, prev));

    if (d1 <= d2) {
        result.pulley_deg     = theta1;
        result.solution_index = 1;
    } else {
        result.pulley_deg     = theta2;
        result.solution_index = 2;
    }

    /* 7. 脉冲增量 */
    abs_target  = FourBar_AngleToPulses(result.pulley_deg);
    result.pulses = abs_target - FourBar_AngleToPulses(prev);

    result.rocker_phi_deg = phi_deg;
    s_last_theta_deg = result.pulley_deg;

    return result;
}

int32_t FourBar_AngleToPulses(float angle_deg)
{
    float n = norm_360(angle_deg);
    return (int32_t)roundf(n / 360.0f * (float)FOURBAR_STEPS_PER_REV);
}

float FourBar_PulsesToAngle(int32_t pulses)
{
    return (float)pulses * 360.0f / (float)FOURBAR_STEPS_PER_REV;
}

/**
 * @brief   正解：电机转角 θ → 摆杆倾角 φ
 *
 * @details 在 O-centered 坐标系:
 *          A' = (-Ox, -Oy),  B' = r·(cosθ, sinθ)
 *          C' = A' + R·(cosφ, sinφ)
 *          |C' - B'| = L
 *
 *          设 u = A'x - B'x, v = A'y - B'y
 *          K = (L² - u² - v² - R²) / (2R)
 *          β = atan2(v, u)
 *          φ = β ± acos(K/sqrt(u²+v²))
 */
float FourBar_Forward(float theta_deg)
{
    float theta_rad, Bpx, Bpy, u, v, D2, D_val, K, cos_val;
    float beta, gamma, phi1, phi2;

    if (!s_initialized) {
        FourBar_Init();
    }

    /* B 在 O-centered */
    theta_rad = DEG2RAD(theta_deg);
    Bpx = FOURBAR_r_MM * cosf(theta_rad);
    Bpy = FOURBAR_r_MM * sinf(theta_rad);

    /* A' = A - O = (-Ox, -Oy) */
    u = -FOURBAR_OX_MM - Bpx;
    v = -FOURBAR_OY_MM - Bpy;

    D2 = SQ(u) + SQ(v);
    D_val = sqrtf(D2);

    K = (SQ(FOURBAR_L_MM) - D2 - SQ(FOURBAR_R_MM))
        / (2.0f * FOURBAR_R_MM);

    cos_val = K / D_val;

    if (cos_val > 1.0f || cos_val < -1.0f) {
        return NAN;
    }

    beta  = atan2f(v, u);
    gamma = acosf(cos_val);

    phi1 = RAD2DEG(beta + gamma);
    phi2 = RAD2DEG(beta - gamma);

    /* 返回与当前逆解状态一致的 phi
     *   简易策略：返回 phi1。
     *   若要精确匹配需要额外记录装配模式。 */
    (void)phi2;
    return norm_360(phi1);
}
