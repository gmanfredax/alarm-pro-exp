/*
 * pdo_slave.c
 *
 *  Created on: Oct 12, 2025
 *      Author: gabriele
 */


#include "pdo_slave.h"

#include "heartbeat_slave.h"
#include "led_ctrl.h"
#include "pins.h"
#include "sdo_slave.h"

#include <string.h>

#include <stdbool.h>

#include "stm32f1xx_hal.h"

#define PDO_TX1_BASE   0x180U
#define PDO_RX1_BASE   0x200U
#define PDO_RX2_BASE   0x300U

static uint8_t current_node_id = 0xFFU;
static uint32_t last_inputs_bitmap = 0U;
static uint8_t change_counter = 0U;
static uint32_t last_tx_time = 0U;

static void apply_output_bitmap(uint32_t bitmap, uint8_t pwm_level)
{
    for (uint8_t i = 0; i < OUTPUT_CHANNEL_COUNT; ++i)
    {
        GPIO_PinState state = (bitmap & (1UL << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        const output_config_t *cfg = SDO_Slave_GetOutputConfig(i);
        if (cfg->type == OUTPUT_TYPE_DIGITAL)
        {
            HAL_GPIO_WritePin(OUTPUT_PORTS[i], OUTPUT_PINS[i], state);
        }
        else
        {
            bool enabled = (state == GPIO_PIN_SET) && (pwm_level > 0U);
            HAL_GPIO_WritePin(OUTPUT_PORTS[i], OUTPUT_PINS[i], enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
}

static void publish_inputs(uint32_t bitmap)
{
    if (current_node_id == 0xFFU)
    {
        return;
    }
    can_frame_t frame = {0};
    frame.id = PDO_TX1_BASE + current_node_id;
    frame.dlc = 8;
    memcpy(&frame.data[0], &bitmap, sizeof(bitmap));
    frame.data[4] = change_counter;
    CAN_Bus_Send(&frame);
    last_tx_time = HAL_GetTick();
}

void PDO_Slave_Init(void)
{
    last_inputs_bitmap = 0U;
    change_counter = 0U;
    last_tx_time = 0U;
}

void PDO_Slave_SetNodeId(uint8_t node_id)
{
    current_node_id = node_id;
}

void PDO_Slave_OnInputChange(uint32_t bitmap, uint8_t counter)
{
    change_counter = counter;
    last_inputs_bitmap = bitmap;
    publish_inputs(bitmap);
}

void PDO_Slave_Task(uint32_t now_ms)
{
    if (current_node_id == 0xFFU)
    {
        return;
    }
    if ((now_ms - last_tx_time) >= 1000U)
    {
        publish_inputs(last_inputs_bitmap);
    }
}

void PDO_Slave_OnFrame(const can_frame_t *frame)
{
    if (current_node_id == 0xFFU)
    {
        return;
    }
    if (frame->id == (PDO_RX1_BASE + current_node_id) && frame->dlc >= 5)
    {
        uint32_t bitmap;
        memcpy(&bitmap, &frame->data[0], sizeof(bitmap));
        uint8_t pwm = frame->data[4];
        apply_output_bitmap(bitmap, pwm);
    }
    else if (frame->id == (PDO_RX2_BASE + current_node_id) && frame->dlc >= 4)
    {
        uint8_t cmd = frame->data[0];
        uint16_t duration;
        memcpy(&duration, &frame->data[1], sizeof(duration));
        uint8_t pattern = frame->data[3];
        LED_Ctrl_Command(cmd, duration, pattern);
    }
}
