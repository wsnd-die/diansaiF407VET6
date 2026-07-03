#include "bujin_can.h"
#include "can.h"
#include "odometer.h"

/* ──────────────────────────────────────────────
   Emm V5 步进驱动器 — CAN 传输层版本
   协议说明书 6.4 节：
   - 帧类型：扩展帧 (29-bit ID)
   - CAN ID = (ID_Addr << 8) | 包序号
   - 数据负载 = 原始 UART 帧去除地址字节
   - >8 字节命令：拆包后每包均重复功能码
   ────────────────────────────────────────────── */

#define BUJIN_PI                  3.1415926f
#define BUJIN_PULSE_PER_REV       3200.0f
#define BUJIN_WHEEL_RADIUS_MM     85.0f

/* ── 私有：CAN 发送（协议 6.4 节） ── */

/**
  * @brief  通过 CAN1 扩展帧发送 Emm V5 指令
  * @param  data  完整 UART 帧（含地址字节 data[0]）
  * @param  len   帧总长度
  * @note   CAN ID = (ID_Addr << 8) | 包序号
  *         地址字节在 CAN ID 中编码，数据负载不含地址
  *         >8 字节时每包均重复功能码（data[1]，即原帧第二字节）
  */
static void Emm_Send(const uint8_t *data, uint16_t len)
{
    const uint8_t *payload     = data + 1;        /* 跳过地址字节 */
    uint16_t       payload_len = len - 1;

    if (payload_len <= 8)
    {
        /* 单帧：ID = (ID_Addr << 8) | 0x00 */
        CAN_SendMessageExt(((uint32_t)EMM_CAN_ID_ADDR << 8) | 0x00U,
                           (uint8_t *)payload, payload_len);
    }
    else
    {
        /*
         * 拆包（参见说明书大于 8 字节位置模式示例）：
         * - 第 0 包：功能码 + 后续 7 字节，ID 低 8 位 = 0x00
         * - 第 1 包：功能码（重复）+ 剩余字节，ID 低 8 位 = 0x01
         */
        uint8_t        cmd_byte = payload[0];           /* 功能码，如 0xFD */
        const uint8_t *tail     = payload + 1;           /* 功能码之后的字节 */
        uint16_t       tail_len = payload_len - 1;
        uint8_t        pkt[8];

        /* Packet 0: 功能码 + tail[0..6] = 8 字节 */
        pkt[0] = cmd_byte;
        memcpy(pkt + 1, tail, 7);
        CAN_SendMessageExt(((uint32_t)EMM_CAN_ID_ADDR << 8) | 0x00U, pkt, 8);

        /* Packet 1: 功能码 + tail[7..] = 变长 */
        uint16_t pkt1_tail_len = tail_len - 7;
        pkt[0] = cmd_byte;
        memcpy(pkt + 1, tail + 7, pkt1_tail_len);
        CAN_SendMessageExt(((uint32_t)EMM_CAN_ID_ADDR << 8) | 0x01U, pkt, 1 + pkt1_tail_len);
    }
}

/* ── CAN 接收分发：由 can.c 的 HAL_CAN_RxFifo0MsgPendingCallback 调用 ── */

/**
  * @brief  CAN 接收分发：匹配本电机 ID_Addr 的扩展帧 → 补地址字节 → 送入里程计解析
  * @param  can_id : CAN 帧 ID（已按 IDE 取 ExtId 或 StdId）
  * @param  data   : 帧数据（不含地址字节）
  * @param  len    : 数据长度
  * @note   驱动器响应帧不含地址字节（已编码在 CAN ID 中），
  *         而 Odometer_UartRxByte 期望首字节为地址，故在数据前补发地址字节
  */
void Bujin_CAN_RxDispatch(uint32_t can_id, uint8_t *data, uint8_t len)
{
    uint8_t addr = (uint8_t)((can_id >> 8) & 0xFFU);

    /* 仅处理本电机 ID_Addr 的帧 */
    if (addr != EMM_CAN_ID_ADDR || data == NULL || len == 0)
        return;

    /* 补地址字节，复用 UART 版里程计解析 */
    Odometer_UartRxByte(EMM_CAN_ID_ADDR);
    for (uint8_t i = 0; i < len; i++)
    {
        Odometer_UartRxByte(data[i]);
    }
}

/* ── 私有：毫米 → 脉冲换算 ── */

static uint32_t Emm_MmToPulse(float mm)
{
    float pulse;

    if (mm <= 0.0f)
    {
        return 0;
    }

    pulse = mm / (2.0f * BUJIN_PI * BUJIN_WHEEL_RADIUS_MM) * BUJIN_PULSE_PER_REV;
    return (uint32_t)(pulse + 0.5f);
}

/* ── 私有：按脉冲数位置控制（13 字节帧，会拆为 2 帧 CAN） ── */

static void Emm_V5_Pos_Control_ByPulse(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                                        uint32_t pulse, bool raF, bool snF)
{
    /* 帧格式：地址 + 0xFD + 方向 + 速度 + 加速度 + 4 字节脉冲 + 相对/绝对 + 同步 + 0x6B */
    uint8_t cmd[13] = {
        addr,
        0xFD,
        dir,
        (uint8_t)(vel >> 8),
        (uint8_t)(vel >> 0),
        acc,
        (uint8_t)(pulse >> 24),
        (uint8_t)(pulse >> 16),
        (uint8_t)(pulse >> 8),
        (uint8_t)(pulse >> 0),
        (uint8_t)raF,
        (uint8_t)snF,
        0x6B,
    };

    Emm_Send(cmd, sizeof(cmd));
    HAL_Delay(5);
}

/* ══════════════════════════════════════════════
   公有 API（与 bujin.c 接口完全一致）
   ══════════════════════════════════════════════ */

/**
  * @brief  电机使能控制
  * @param  addr  电机地址
  * @param  state true 使能，false 关闭
  * @param  snF   同步标志
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t cmd[6] = {addr, 0xF3, 0xAB, (uint8_t)state, (uint8_t)snF, 0x6B};
    Emm_Send(cmd, sizeof(cmd));
}

/**
  * @brief  立即停止电机
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t cmd[5] = {addr, 0xFE, 0x98, (uint8_t)snF, 0x6B};
    Emm_Send(cmd, sizeof(cmd));
}

/**
  * @brief  速度模式控制
  * @param  vel  RPM
  * @param  acc  加速度，0 = 直接启动
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[8] = {
        addr,
        0xF6,
        dir,
        (uint8_t)(vel >> 8),
        (uint8_t)(vel >> 0),
        acc,
        (uint8_t)snF,
        0x6B,
    };

    Emm_Send(cmd, sizeof(cmd));
    HAL_Delay(1);
}

/**
  * @brief  位置模式控制（mm → 脉冲自动换算）
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                         float mm, bool raF, bool snF)
{
    Emm_V5_Pos_Control_ByPulse(addr, dir, vel, acc, Emm_MmToPulse(mm), raF, snF);
}

/**
  * @brief  修改驱动器控制模式
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[6] = {addr, 0x46, 0x69, (uint8_t)svF, ctrl_mode, 0x6B};
    Emm_Send(cmd, sizeof(cmd));
}

/**
  * @brief  清除堵转保护
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0E, 0x52, 0x6B};
    Emm_Send(cmd, sizeof(cmd));
}

/**
  * @brief  触发同步运动（先以 snF=true 下发动作，再调用本函数触发）
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0xFF, 0x66, 0x6B};
    Emm_Send(cmd, sizeof(cmd));
    HAL_Delay(1);
}

/**
  * @brief  读取驱动器系统参数
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t cmd[4] = {0};
    uint8_t i = 0;

    cmd[i++] = addr;
    switch (s)
    {
        case S_VER:   cmd[i++] = 0x1F; break;
        case S_RL:    cmd[i++] = 0x20; break;
        case S_PID:   cmd[i++] = 0x21; break;
        case S_VBUS:  cmd[i++] = 0x24; break;
        case S_CPHA:  cmd[i++] = 0x27; break;
        case S_ENCL:  cmd[i++] = 0x31; break;
        case S_TPOS:  cmd[i++] = 0x33; break;
        case S_VEL:   cmd[i++] = 0x35; break;
        case S_CPOS:  cmd[i++] = 0x36; break;
        case S_PERR:  cmd[i++] = 0x37; break;
        case S_FLAG:  cmd[i++] = 0x3A; break;
        case S_ORG:   cmd[i++] = 0x3B; break;
        case S_Conf:  cmd[i++] = 0x42; cmd[i++] = 0x6C; break;
        case S_State: cmd[i++] = 0x43; cmd[i++] = 0x7A; break;
        default: return;
    }

    cmd[i++] = 0x6B;
    Emm_Send(cmd, i);
}

/**
  * @brief  按角度控制电机
  */
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc)
{
    uint8_t dir = 0;

    if (angle < 0.0f)
    {
        angle = -angle;
        dir = 1;
    }

    Emm_V5_Pos_Control_ByPulse(addr, dir, vel, acc,
        (uint32_t)(BUJIN_PULSE_PER_REV * angle / 360.0f), false, false);
}
