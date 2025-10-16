#include "hw_profile.h"

#include <string.h>

#define OUTPUT_BASE_PIN  GPIO_PIN_8
#define IDENTIFY_TICKS   15u

static bool s_identify_enabled = false;
static bool s_identify_state = false;
static uint32_t s_identify_counter = 0;

uint32_t hw_profile_read_inputs(void)
{
    uint32_t value = 0;
    value |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET ? 0u : 1u) << 0;
    value |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET ? 0u : 1u) << 1;
    value |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET ? 0u : 1u) << 2;
    value |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET ? 0u : 1u) << 3;
    value |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET ? 0u : 1u) << 4;
    value |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET ? 0u : 1u) << 5;
    value |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET ? 0u : 1u) << 6;
    value |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET ? 0u : 1u) << 7;
    return value;
}

void hw_profile_write_outputs(uint32_t bitmap)
{
    for (uint8_t idx = 0; idx < EXP_BOARD_OUTPUT_COUNT; ++idx) {
        uint16_t pin = OUTPUT_BASE_PIN << idx;
        GPIO_PinState state = (bitmap & (1u << idx)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(GPIOB, pin, state);
    }
}

void hw_profile_identify(bool enable)
{
    s_identify_enabled = enable;
    s_identify_counter = 0;
    if (!enable) {
        s_identify_state = false;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

void hw_profile_tick_10ms(void)
{
    if (!s_identify_enabled) {
        return;
    }

    if (++s_identify_counter >= IDENTIFY_TICKS) {
        s_identify_counter = 0;
        s_identify_state = !s_identify_state;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, s_identify_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

void hw_profile_get_unique_id(uint8_t *out_uid, size_t len)
{
    if (!out_uid || len == 0u) {
        return;
    }

    const uint32_t *uid_regs = (const uint32_t *)0x1FFFF7E8u;
    uint8_t buffer[12] = {0};

    for (size_t i = 0; i < 3; ++i) {
        uint32_t word = uid_regs[i];
        buffer[(i * 4u) + 0u] = (uint8_t)(word & 0xFFu);
        buffer[(i * 4u) + 1u] = (uint8_t)((word >> 8) & 0xFFu);
        buffer[(i * 4u) + 2u] = (uint8_t)((word >> 16) & 0xFFu);
        buffer[(i * 4u) + 3u] = (uint8_t)((word >> 24) & 0xFFu);
    }

    size_t copy_len = (len < sizeof(buffer)) ? len : sizeof(buffer);
    memcpy(out_uid, buffer, copy_len);
    if (len > copy_len) {
        memset(out_uid + copy_len, 0, len - copy_len);
    }
}
