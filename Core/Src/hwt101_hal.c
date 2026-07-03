#include "hwt101_hal.h"

/* 引用 main.c 中的 UART4 句柄 */
extern UART_HandleTypeDef huart4;

/* ---- 直接解析输出的角度（归一化 int16, ±32768 → ±180°） ---- */
volatile float g_hwt101_roll  = 0.0f;
volatile float g_hwt101_pitch = 0.0f;
volatile float g_hwt101_yaw   = 0.0f;
volatile uint8_t g_hwt101_data_ready = 0;

/* ---- 回调实现：发送 ---- */
static void HWT101_SerialWrite(uint8_t *p_ucData, uint32_t uiLen)
{
    HAL_UART_Transmit(&huart4, p_ucData, uiLen, 100);
}

/* ---- 回调实现：延时 ---- */
static void HWT101_DelayMs(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

/* ---- 回调实现：寄存器更新通知（默认空实现） ---- */
static void HWT101_RegUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    (void)uiReg;
    (void)uiRegNum;
}

/* ---- 包头 + 校验和 ---- */
static uint8_t HWT101_CalcChecksum(uint8_t *data, uint16_t length)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length; i++) sum += data[i];
    return sum;
}

/* ---- 解析一帧 11 字节数据包（由 UART4_IRQHandler 调用） ---- */
void HWT101_ParsePacket(uint8_t *data)
{
    uint8_t cksum;
    int16_t raw;

    /* 校验和 */
    cksum = HWT101_CalcChecksum(data, 10);
    if (cksum != data[10]) return;

    if (data[0] == 0x55 && data[1] == 0x53)         /* 角度包 */
    {
        /* Roll:  data[2]L + data[3]H */
        raw = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
        g_hwt101_roll = ((float)raw / 32768.0f) * 180.0f;

        /* Pitch: data[4]L + data[5]H */
        raw = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
        g_hwt101_pitch = ((float)raw / 32768.0f) * 180.0f;

        /* Yaw:   data[6]L + data[7]H */
        raw = (int16_t)(((uint16_t)data[7] << 8) | data[6]);
        g_hwt101_yaw = ((float)raw / 32768.0f) * 180.0f;

        g_hwt101_data_ready = 1;
    }
    /* 角速度包（0x52）暂不处理，保持函数扩展性 */
}

/* ---- 初始化 ---- */
int32_t HWT101_HAL_Init(void)
{
    int32_t ret;

    /* 1. 注册串口发送回调 */
    ret = WitSerialWriteRegister(HWT101_SerialWrite);
    if (ret != WIT_HAL_OK) return ret;

    /* 2. 注册延时回调 */
    ret = WitDelayMsRegister(HWT101_DelayMs);
    if (ret != WIT_HAL_OK) return ret;

    /* 3. 注册数据更新回调 */
    ret = WitRegisterCallBack(HWT101_RegUpdate);
    if (ret != WIT_HAL_OK) return ret;

    /* 4. 初始化 SDK：Normal 协议，地址 0x50 */
    ret = WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    if (ret != WIT_HAL_OK) return ret;

    /* 5. 设置输出内容：加速度 + 角速度 + 角度 */
    ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);
    if (ret != WIT_HAL_OK) return ret;

    /* 6. 设置输出频率：10Hz */
    ret = WitSetOutputRate(RRATE_10HZ);
    if (ret != WIT_HAL_OK) return ret;

    /* RX 由 UART4_IRQHandler 直接解析，不需要 HAL_UART_Receive_IT */
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);

    return WIT_HAL_OK;
}

/* ---- 便捷访问函数（归一化缩放，±180°） ---- */
float HWT101_GetRoll(void)
{
    return g_hwt101_roll;
}

float HWT101_GetPitch(void)
{
    return g_hwt101_pitch;
}

float HWT101_GetYaw(void)
{
    return g_hwt101_yaw;
}

int16_t HWT101_ReadReg(uint32_t reg)
{
    if (reg >= REGSIZE) return 0;
    return sReg[reg];
}
