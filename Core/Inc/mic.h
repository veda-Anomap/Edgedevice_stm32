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

/* Stage-1 TDOA skeleton debug container.
 * Real GCC-PHAT/angle estimator will be added in later steps. */
typedef struct {
    int32_t lag_samples;
    int32_t tau_us;
    int32_t alpha_deg_x10;
    uint16_t peak_main;
    uint16_t peak_second;
    uint16_t confidence_q8;
    uint8_t vad_pass;
    uint8_t valid;
} mic_tdoa_debug_t;

void mic_init(void);
#ifdef HAL_ADC_MODULE_ENABLED
void mic_on_dma_complete(ADC_HandleTypeDef *hadc);
#endif
#ifdef HAL_I2S_MODULE_ENABLED
void mic_on_i2s_rx_complete(I2S_HandleTypeDef *hi2s);
#endif
char mic_process(uint32_t now_ms, uint32_t motor_lock_until_ms);
uint8_t mic_is_calibrated(void);
void mic_get_debug(mic_debug_t *out);

/* Stage-1 TDOA API skeleton */
void mic_tdoa_enable(uint8_t enable);
void mic_tdoa_process(uint32_t now_ms);
uint8_t mic_tdoa_is_valid(void);
void mic_get_tdoa_debug(mic_tdoa_debug_t *out);
