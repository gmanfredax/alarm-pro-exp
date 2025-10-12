/*
 * sdo_slave.c
 *
 *  Created on: Oct 12, 2025
 *      Author: gabriele
 */


#include "sdo_slave.h"

#include "can_bus_stm32.h"
#include "eeprom_emul.h"
#include "pins.h"

#include <string.h>

#define SDO_RX_BASE 0x600U
#define SDO_TX_BASE 0x580U

static uint8_t node_id = 0xFFU;
static input_config_t input_config[INPUT_CHANNEL_COUNT];
static output_config_t output_config[OUTPUT_CHANNEL_COUNT];
static board_info_t board_info;

static void send_abort(uint16_t index, uint8_t subindex, uint32_t code)
{
    can_frame_t frame = {0};
    frame.id = SDO_TX_BASE + node_id;
    frame.dlc = 8;
    frame.data[0] = 0x80;
    frame.data[1] = (uint8_t)(index & 0xFFU);
    frame.data[2] = (uint8_t)((index >> 8) & 0xFFU);
    frame.data[3] = subindex;
    memcpy(&frame.data[4], &code, sizeof(code));
    CAN_Bus_Send(&frame);
}

static void send_upload(uint16_t index, uint8_t subindex, const void *data, uint8_t len)
{
    can_frame_t frame = {0};
    frame.id = SDO_TX_BASE + node_id;
    frame.dlc = 8;
    uint8_t n = (uint8_t)(4U - len);
    frame.data[0] = 0x43 | (n << 2) | 0x02;
    frame.data[1] = (uint8_t)(index & 0xFFU);
    frame.data[2] = (uint8_t)((index >> 8) & 0xFFU);
    frame.data[3] = subindex;
    memcpy(&frame.data[4], data, len);
    CAN_Bus_Send(&frame);
}

static void send_download_ack(uint16_t index, uint8_t subindex)
{
    can_frame_t frame = {0};
    frame.id = SDO_TX_BASE + node_id;
    frame.dlc = 8;
    frame.data[0] = 0x60;
    frame.data[1] = (uint8_t)(index & 0xFFU);
    frame.data[2] = (uint8_t)((index >> 8) & 0xFFU);
    frame.data[3] = subindex;
    CAN_Bus_Send(&frame);
}

static bool get_input_entry(uint8_t subindex, uint32_t *value)
{
    if (subindex == 0U)
    {
        *value = INPUT_CHANNEL_COUNT;
        return true;
    }
    uint8_t channel = subindex - 1U;
    if (channel >= INPUT_CHANNEL_COUNT)
    {
        return false;
    }
    const input_config_t *cfg = &input_config[channel];
    *value = ((uint32_t)cfg->type & 0xFFU)
             | (((uint32_t)cfg->debounce_ms & 0xFFFFU) << 8)
             | ((uint32_t)(cfg->inverted ? 1U : 0U) << 24);
    return true;
}

static bool get_output_entry(uint8_t subindex, uint32_t *value)
{
    if (subindex == 0U)
    {
        *value = OUTPUT_CHANNEL_COUNT;
        return true;
    }
    uint8_t channel = subindex - 1U;
    if (channel >= OUTPUT_CHANNEL_COUNT)
    {
        return false;
    }
    const output_config_t *cfg = &output_config[channel];
    *value = ((uint32_t)cfg->type & 0xFFU) | ((uint32_t)cfg->default_pwm << 8);
    return true;
}

static bool set_input_entry(uint8_t subindex, uint32_t value)
{
    if (subindex == 0U)
    {
        return false;
    }
    uint8_t channel = subindex - 1U;
    if (channel >= INPUT_CHANNEL_COUNT)
    {
        return false;
    }
    input_config[channel].type = (input_type_t)(value & 0xFFU);
    input_config[channel].debounce_ms = (uint16_t)((value >> 8) & 0xFFFFU);
    input_config[channel].inverted = (((value >> 24) & 0x1U) != 0U);
    EEPROM_SaveInputConfig(input_config, INPUT_CHANNEL_COUNT);
    return true;
}

static bool set_output_entry(uint8_t subindex, uint32_t value)
{
    if (subindex == 0U)
    {
        return false;
    }
    uint8_t channel = subindex - 1U;
    if (channel >= OUTPUT_CHANNEL_COUNT)
    {
        return false;
    }
    output_config[channel].type = (output_type_t)(value & 0xFFU);
    output_config[channel].default_pwm = (uint8_t)((value >> 8) & 0xFFU);
    EEPROM_SaveOutputConfig(output_config, OUTPUT_CHANNEL_COUNT);
    return true;
}

void SDO_Slave_Init(void)
{
    EEPROM_LoadBoardInfo(&board_info);
    EEPROM_LoadInputConfig(input_config, INPUT_CHANNEL_COUNT);
    EEPROM_LoadOutputConfig(output_config, OUTPUT_CHANNEL_COUNT);
}

void SDO_Slave_LoadDefaults(void)
{
    for (uint8_t i = 0; i < INPUT_CHANNEL_COUNT; ++i)
    {
        input_config[i].type = INPUT_TYPE_NC;
        input_config[i].debounce_ms = 30U;
        input_config[i].inverted = false;
    }
    for (uint8_t i = 0; i < OUTPUT_CHANNEL_COUNT; ++i)
    {
        output_config[i].type = OUTPUT_TYPE_DIGITAL;
        output_config[i].default_pwm = 0U;
    }
    EEPROM_SaveInputConfig(input_config, INPUT_CHANNEL_COUNT);
    EEPROM_SaveOutputConfig(output_config, OUTPUT_CHANNEL_COUNT);
}

void SDO_Slave_SetNodeId(uint8_t id)
{
    node_id = id;
}

const input_config_t *SDO_Slave_GetInputConfig(uint8_t channel)
{
    return (channel < INPUT_CHANNEL_COUNT) ? &input_config[channel] : NULL;
}

const output_config_t *SDO_Slave_GetOutputConfig(uint8_t channel)
{
    return (channel < OUTPUT_CHANNEL_COUNT) ? &output_config[channel] : NULL;
}

void SDO_Slave_OnFrame(const can_frame_t *frame)
{
    if (node_id == 0xFFU)
    {
        return;
    }
    if (frame->id != (SDO_RX_BASE + node_id))
    {
        return;
    }
    if (frame->dlc != 8)
    {
        return;
    }

    uint8_t cmd = frame->data[0];
    uint16_t index = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8);
    uint8_t subindex = frame->data[3];

    if (cmd == 0x40)
    {
        uint32_t value = 0U;
        bool ok = false;
        switch (index)
        {
        case 0x2000:
            if (subindex == 1)
            {
                value = board_info.model;
                ok = true;
            }
            else if (subindex == 2)
            {
                value = board_info.fw_version;
                ok = true;
            }
            else if (subindex == 3)
            {
                value = board_info.caps;
                ok = true;
            }
            else if (subindex == 4)
            {
                value = board_info.inputs;
                ok = true;
            }
            else if (subindex == 5)
            {
                value = board_info.outputs;
                ok = true;
            }
            break;
        case 0x2100:
            ok = get_input_entry(subindex, &value);
            break;
        case 0x2200:
            ok = get_output_entry(subindex, &value);
            break;
        default:
            break;
        }
        if (ok)
        {
            send_upload(index, subindex, &value, sizeof(value));
        }
        else
        {
            send_abort(index, subindex, 0x06020000U);
        }
    }
    else if (cmd == 0x23 || cmd == 0x2B || cmd == 0x2F)
    {
        uint32_t value = 0U;
        memcpy(&value, &frame->data[4], sizeof(value));
        bool ok = false;
        switch (index)
        {
        case 0x2100:
            ok = set_input_entry(subindex, value);
            break;
        case 0x2200:
            ok = set_output_entry(subindex, value);
            break;
        default:
            break;
        }
        if (ok)
        {
            send_download_ack(index, subindex);
        }
        else
        {
            send_abort(index, subindex, 0x06090011U);
        }
    }
    else
    {
        send_abort(index, subindex, 0x05040001U);
    }
}
