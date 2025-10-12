/*
 * can_bus_stm32.h
 *
 *  Created on: Oct 12, 2025
 *      Author: gabriele
 */

#ifndef INC_CAN_BUS_STM32_H_
#define INC_CAN_BUS_STM32_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

void CAN_Bus_Init(void);
bool CAN_Bus_Send(const can_frame_t *frame);
bool CAN_Bus_Read(can_frame_t *frame);
void CAN_Bus_Start(void);
void CAN_Bus_SetFilters(uint8_t node_id);
CAN_HandleTypeDef *CAN_Bus_GetHandle(void);

#endif /* INC_CAN_BUS_STM32_H_ */
