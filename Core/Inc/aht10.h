#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    int32_t temperature_c_x100; /* 2534 -> 25.34 C */
    uint32_t humidity_rh_x100;  /* 4567 -> 45.67 %RH */
    uint8_t valid;              /* whether at least one valid sample exists */
    uint8_t busy;               /* conversion in progress flag */
    uint32_t period_ms;         /* measurement period in milliseconds */
    uint32_t last_update_ms;    /* last valid update tick (HAL_GetTick) */
    uint32_t error_count;       /* accumulated communication/parse errors */
} aht10_data_t;

void aht10_init(I2C_HandleTypeDef *hi2c, uint32_t period_ms);
void aht10_process(uint32_t now_ms);
void aht10_get_data(aht10_data_t *out);
