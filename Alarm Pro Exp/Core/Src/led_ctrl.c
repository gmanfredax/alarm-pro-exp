/*
 * led_ctrl.c
 *
 *  Created on: Oct 12, 2025
 *      Author: gabriele
 */


#include "led_ctrl.h"

#include "pins.h"

#include "stm32f1xx_hal.h"

#include <stdbool.h>

typedef enum
{
    LED_MODE_OFF = 0,
    LED_MODE_IDENTIFY,
    LED_MODE_PATTERN
} led_mode_t;

static led_mode_t current_mode = LED_MODE_OFF;
static uint32_t next_transition = 0U;
static uint8_t remaining_blinks = 0U;
static uint32_t on_duration = 0U;
static uint32_t off_duration = 0U;
static bool led_state = false;

static void set_led(bool on)
{
    HAL_GPIO_WritePin(LED_IDENTIFY_PORT, LED_IDENTIFY_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    led_state = on;
}

void LED_Ctrl_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = LED_IDENTIFY_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_IDENTIFY_PORT, &gpio);
    set_led(false);
    current_mode = LED_MODE_OFF;
    next_transition = 0U;
    remaining_blinks = 0U;
}

void LED_Ctrl_SetIdentify(bool enable)
{
    if (enable)
    {
        current_mode = LED_MODE_IDENTIFY;
        on_duration = 250U;
        off_duration = 250U;
        remaining_blinks = 0xFFU;
        set_led(true);
        next_transition = HAL_GetTick() + on_duration;
    }
    else
    {
        current_mode = LED_MODE_OFF;
        set_led(false);
    }
}

void LED_Ctrl_Command(uint8_t cmd, uint16_t duration_ms, uint8_t pattern_arg)
{
    switch (cmd)
    {
    case 0x00: // OFF
        current_mode = LED_MODE_OFF;
        set_led(false);
        break;
    case 0x01: // BLINK_ONESHOT
        current_mode = LED_MODE_PATTERN;
        if (pattern_arg == 1U)
        {
            remaining_blinks = 3U;
            on_duration = 100U;
            off_duration = 100U;
        }
        else if (pattern_arg == 2U)
        {
            remaining_blinks = 2U;
            on_duration = 300U;
            off_duration = 300U;
        }
        else
        {
            remaining_blinks = 1U;
            on_duration = (duration_ms > 0U) ? duration_ms : 200U;
            off_duration = on_duration;
        }
        set_led(true);
        next_transition = HAL_GetTick() + on_duration;
        break;
    case 0x02: // IDENTIFY_TOGGLE
        LED_Ctrl_SetIdentify(true);
        break;
    case 0x03: // BLINK_CUSTOM
        current_mode = LED_MODE_PATTERN;
        remaining_blinks = (duration_ms > 0U) ? (duration_ms / 200U) : 3U;
        if (remaining_blinks == 0U)
        {
            remaining_blinks = 3U;
        }
        on_duration = 200U;
        off_duration = 200U;
        set_led(true);
        next_transition = HAL_GetTick() + on_duration;
        break;
    default:
        break;
    }
}

void LED_Ctrl_Task(uint32_t now_ms)
{
    if (current_mode == LED_MODE_OFF)
    {
        return;
    }
    if (now_ms < next_transition)
    {
        return;
    }
    if (current_mode == LED_MODE_IDENTIFY)
    {
        set_led(!led_state);
        next_transition = now_ms + (led_state ? on_duration : off_duration);
    }
    else if (current_mode == LED_MODE_PATTERN)
    {
        set_led(!led_state);
        if (led_state)
        {
            next_transition = now_ms + on_duration;
        }
        else
        {
            next_transition = now_ms + off_duration;
            if (remaining_blinks > 0U)
            {
                --remaining_blinks;
            }
            if (remaining_blinks == 0U)
            {
                current_mode = LED_MODE_OFF;
                set_led(false);
            }
        }
    }
}
