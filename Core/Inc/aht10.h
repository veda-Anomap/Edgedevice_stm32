#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    int32_t temperature_c_x100; /* 2534 -> 25.34 C */
    uint32_t humidity_rh_x100;  /* 4567 -> 45.67 %RH */
    uint8_t valid;              /* 첫 유효 샘플 확보 여부 */
    uint8_t busy;               /* 변환 진행 중 여부 */
    uint32_t period_ms;         /* 측정 주기(ms) */
    uint32_t last_update_ms;    /* 마지막 유효 갱신 시각(HAL_GetTick) */
    uint32_t error_count;       /* 통신/파싱 오류 누적 횟수 */
} aht10_data_t;

void aht10_init(I2C_HandleTypeDef *hi2c, uint32_t period_ms);
void aht10_process(uint32_t now_ms);
void aht10_get_data(aht10_data_t *out);
