/*
 * can_bus_stm32.c
 *
 *  Created on: Oct 12, 2025
 *      Author: gabriele
 */


#include "can_bus_stm32.h"

#include "main.h"

#include <stdbool.h>

static CAN_HandleTypeDef hcan;
static uint8_t current_node_id = 0xFFU;

void CAN_Bus_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 6;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = ENABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = ENABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_Bus_SetFilters(uint8_t node_id)
{
    current_node_id = node_id;
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_Bus_Start(void)
{
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

bool CAN_Bus_Send(const can_frame_t *frame)
{
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = frame->id & 0x7FFU;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame->dlc;

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan, &header, (uint8_t *)frame->data, &mailbox) != HAL_OK)
    {
        return false;
    }
    while (HAL_CAN_IsTxMessagePending(&hcan, mailbox))
    {
    }
    return true;
}

bool CAN_Bus_Read(can_frame_t *frame)
{
    CAN_RxHeaderTypeDef header = {0};
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, frame->data) != HAL_OK)
    {
        return false;
    }
    frame->id = header.StdId;
    frame->dlc = header.DLC;
    return true;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    (void)hcan_ptr;
}

CAN_HandleTypeDef *CAN_Bus_GetHandle(void)
{
    return &hcan;
}
