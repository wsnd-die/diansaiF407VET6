/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include <string.h>
#include "bujin_can.h"

/* 接收缓冲区：中断回调中填充，主循环/任务中读取 */
static volatile uint8_t  can_rx_new     = 0;
static volatile uint32_t can_rx_id      = 0;
static volatile uint8_t  can_rx_data[8] = {0};
static volatile uint8_t  can_rx_len     = 0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  配置 CAN1 过滤器：接受所有标准帧和扩展帧
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_FilterConfig_AcceptAll(void)
{
    CAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    return HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

/**
  * @brief  启动 CAN1 并激活中断
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_Start(void)
{
    HAL_StatusTypeDef status;

    status = HAL_CAN_Start(&hcan1);
    if (status != HAL_OK)
        return status;

    /* 使能 FIFO0 消息挂起中断（收到消息时触发回调） */
    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  发送一条 CAN 消息
  * @param  id   : 标准帧 ID (0x000 ~ 0x7FF) 或扩展帧 ID (需配合 IDE 位)
  * @param  data : 数据缓冲区
  * @param  len  : 数据长度 0~8
  * @retval HAL status
  * @note   使用邮箱空闲超时 100ms
  */
HAL_StatusTypeDef CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef   txHeader;
    uint32_t              txMailbox;
    uint8_t               txData[8] = {0};

    if (len > 8) len = 8;

    txHeader.StdId = id;
    txHeader.ExtId = 0;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = len;
    txHeader.TransmitGlobalTime = DISABLE;

    if (data != NULL)
    {
        memcpy(txData, data, len);
    }

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/**
  * @brief  发送一条扩展帧 CAN 消息
  * @param  id   : 29 位扩展帧 ID
  * @param  data : 数据缓冲区
  * @param  len  : 数据长度 0~8
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendMessageExt(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef   txHeader;
    uint32_t              txMailbox;
    uint8_t               txData[8] = {0};

    if (len > 8) len = 8;

    txHeader.ExtId = id;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = len;
    txHeader.TransmitGlobalTime = DISABLE;

    if (data != NULL)
    {
        memcpy(txData, data, len);
    }

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/**
  * @brief  非阻塞获取最近收到的一条 CAN 消息
  * @param  id   : 输出参数，接收到的消息 ID
  * @param  data : 输出参数，接收到的数据（调用者提供 >=8 字节缓冲区）
  * @param  len  : 输出参数，接收到的数据长度
  * @retval 1 = 有新消息  0 = 无新消息
  */
uint8_t CAN_GetReceivedMessage(uint32_t *id, uint8_t *data, uint8_t *len)
{
    if (can_rx_new == 0)
        return 0;

    __disable_irq();  /* 临界区保护 */
    can_rx_new = 0;
    if (id  != NULL) *id  = can_rx_id;
    if (len != NULL) *len = can_rx_len;
    if (data != NULL) memcpy(data, (void *)can_rx_data, can_rx_len);
    __enable_irq();

    return 1;
}

/**
  * @brief  CAN1 FIFO0 消息挂起回调（中断上下文）
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t             rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    /* 扩展帧用 ExtId，标准帧用 StdId */
    {
        uint32_t rx_id = (rxHeader.IDE == CAN_ID_EXT) ? rxHeader.ExtId : rxHeader.StdId;

        /* 电机响应帧 → 分发到里程计解析 */
        Bujin_CAN_RxDispatch(rx_id, rxData, rxHeader.DLC);

        can_rx_new = 1;
        can_rx_id  = rx_id;
        can_rx_len = rxHeader.DLC;
        memcpy((void *)can_rx_data, rxData, can_rx_len > 8 ? 8 : can_rx_len);
    }
}

/**
  * @brief  CAN1 错误回调
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err = HAL_CAN_GetError(hcan);
    /* 预留：可根据 err 值做错误处理，如 CAN_ERROR_EWG / CAN_ERROR_EPV / CAN_ERROR_BOF */
    (void)err;
}

/* ──────────────────────────────────────────────
   CAN 综合测试函数
   ────────────────────────────────────────────── */
void CAN_Test(void)
{
    HAL_StatusTypeDef status;
    uint8_t           test_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t           rx_data[8];
    uint32_t          rx_id;
    uint8_t           rx_len;

    /* 1. 配置过滤器（接收全部） */
    status = CAN_FilterConfig_AcceptAll();
    if (status != HAL_OK)
    {
        /* 过滤器配置失败 — 根据项目实际情况处理 */
        while (1) {}
    }

    /* 2. 启动 CAN1 + 使能 FIFO0 中断通知 */
    status = CAN_Start();
    if (status != HAL_OK)
    {
        /* 启动失败 */
        while (1) {}
    }

    /* 3. 发送一条测试消息 — ID=0x123, DLC=8 */
    status = CAN_SendMessage(0x123, test_data, 8);
    if (status != HAL_OK)
    {
        /* 发送失败 — 检查总线连接 / 终端电阻 */
        /* 错误码参考：HAL_ERROR / HAL_TIMEOUT / HAL_BUSY */
    }

    /* 4. 轮询接收（示例：等待最多 500ms） */
    {
        uint32_t tick = HAL_GetTick();
        while ((HAL_GetTick() - tick) < 500)
        {
            if (CAN_GetReceivedMessage(&rx_id, rx_data, &rx_len))
            {
                /* 收到消息 — rx_id/rx_data/rx_len 有效 */
                /* 实际使用时替换为业务处理逻辑 */
                (void)rx_id;
                (void)rx_data;
                (void)rx_len;
                break;
            }
        }
    }
}

/* USER CODE END 1 */

