#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    uint32_t adc_avg_l;
    uint32_t adc_avg_r;
    uint32_t peak_l;
    uint32_t peak_r;
    uint32_t sig_l;
    uint32_t sig_r;
    uint32_t noise_l;
    uint32_t noise_r;
    uint32_t snr_l_q8;
    uint32_t snr_r_q8;
    uint32_t diff;
    uint8_t gate_sound;
    uint8_t gate_snr;
    uint8_t gate_ratio;
    uint8_t noise_ready;
    char detect_dir;
} mic_debug_t;

void mic_init(ADC_HandleTypeDef *hadc);
void mic_on_dma_complete(ADC_HandleTypeDef *hadc);
char mic_process(uint32_t now_ms, uint32_t motor_lock_until_ms);
uint8_t mic_is_calibrated(void);
void mic_get_debug(mic_debug_t *out);
