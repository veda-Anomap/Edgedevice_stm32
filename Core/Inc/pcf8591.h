#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    uint32_t light_raw;       /* 0~255 */
    uint8_t valid;            /* 1: valid, 0: invalid */
    uint32_t period_ms;       /* sample period */
    uint32_t last_update_ms;  /* HAL_GetTick timestamp */
    uint32_t error_count;     /* I2C errors */
} pcf8591_data_t;

void pcf8591_init(I2C_HandleTypeDef *hi2c, uint32_t period_ms);
void pcf8591_process(uint32_t now_ms);
void pcf8591_get_data(pcf8591_data_t *out);

