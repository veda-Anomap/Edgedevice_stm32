#include "app.h"
#include <stdint.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* DMA buffer: interleaved [L, R, L, R, ...] */
#define ADC_BUF_LEN 200U
uint16_t adc_buffer[ADC_BUF_LEN];

/* Direction detection gates */
#define SOUND_TH       400U
#define DIFF_TH        220U
#define ALPHA_DIV      4U
#define SWITCH_HOLDOFF 40U

/* Sliding windows for SNR */
#define SIG_WIN          6U
#define NOISE_WIN       64U
#define SNR_TH_Q8      384U /* 1.50x in Q8 */
#define SNR_DIFF_TH_Q8  51U /* 0.20x in Q8 */
#define NOISE_FREEZE_Q8 320U /* 1.25x in Q8 */

/* Servo control */
#define SERVO_LEFT_US    1200U
#define SERVO_RIGHT_US   1820U
#define SERVO_CENTER_US  1520U
#define SERVO_RUN_MS      200U
#define DIR_COOLDOWN_MS  1500U

/* Shared state (ISR + main loop) */
static volatile uint32_t baseL = 0U, baseR = 0U;
static volatile uint32_t lvlL = 0U, lvlR = 0U;
static volatile uint8_t is_calibrated = 0U;
static volatile uint32_t peakL = 0U, peakR = 0U;
static volatile uint32_t adcAvgL = 0U, adcAvgR = 0U;
static volatile uint32_t noiseL = 1U, noiseR = 1U;
static volatile uint32_t snrL_q8 = 0U, snrR_q8 = 0U;
static volatile uint8_t noise_ready = 0U;

static char detectLR = '-';
static uint32_t last_switch_ms = 0U;
static uint32_t motor_lock_until_ms = 0U;
static char last_dir = '-';
static uint8_t motor_running = 0U;
static uint32_t motor_stop_ms = 0U;
static char pending_dir = '-';

/* Window accumulators */
static uint16_t sig_histL[SIG_WIN];
static uint16_t sig_histR[SIG_WIN];
static uint16_t noise_histL[NOISE_WIN];
static uint16_t noise_histR[NOISE_WIN];
static uint16_t sig_idx = 0U, sig_count = 0U;
static uint16_t noise_idx = 0U, noise_count = 0U;
static uint32_t sig_sumL = 0U, sig_sumR = 0U;
static uint32_t noise_sumL = 0U, noise_sumR = 0U;

static inline uint32_t u32_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static inline uint32_t q8_ratio(uint32_t num, uint32_t den)
{
    if (den == 0U) den = 1U;
    return (num << 8) / den;
}

static inline void ring_push_pair(uint16_t *histL,
                                  uint16_t *histR,
                                  uint16_t len,
                                  uint16_t *idx,
                                  uint16_t *count,
                                  uint32_t *sumL,
                                  uint32_t *sumR,
                                  uint16_t vL,
                                  uint16_t vR)
{
    if (*count == len) {
        *sumL -= histL[*idx];
        *sumR -= histR[*idx];
    } else {
        (*count)++;
    }

    histL[*idx] = vL;
    histR[*idx] = vR;
    *sumL += vL;
    *sumR += vR;
    *idx = (uint16_t)((*idx + 1U) % len);
}

void app_init(void)
{
    detectLR = '-';
    last_dir = '-';
    pending_dir = '-';
    motor_running = 0U;
    motor_lock_until_ms = 0U;
    motor_stop_ms = 0U;
    last_switch_ms = HAL_GetTick();

    baseL = baseR = 0U;
    lvlL = lvlR = 0U;
    peakL = peakR = 0U;
    adcAvgL = adcAvgR = 0U;
    noiseL = noiseR = 1U;
    snrL_q8 = snrR_q8 = 0U;
    noise_ready = 0U;
    is_calibrated = 0U;

    sig_idx = sig_count = 0U;
    noise_idx = noise_count = 0U;
    sig_sumL = sig_sumR = 0U;
    noise_sumL = noise_sumR = 0U;

    for (uint32_t i = 0U; i < SIG_WIN; i++) {
        sig_histL[i] = 0U;
        sig_histR[i] = 0U;
    }
    for (uint32_t i = 0U; i < NOISE_WIN; i++) {
        noise_histL[i] = 0U;
        noise_histR[i] = 0U;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CENTER_US);
    printf("Starting DMA Audio System...\r\n");
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_LEN);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    const uint32_t samples_per_ch = ADC_BUF_LEN / 2U;

    if (!is_calibrated) {
        uint32_t sumL = 0U, sumR = 0U;
        for (uint32_t i = 0U; i < ADC_BUF_LEN; i += 2U) {
            sumL += adc_buffer[i];
            sumR += adc_buffer[i + 1U];
        }
        baseL = sumL / samples_per_ch;
        baseR = sumR / samples_per_ch;
        is_calibrated = 1U;
        return;
    }

    uint32_t max_magL = 0U, max_magR = 0U;
    uint32_t sum_magL = 0U, sum_magR = 0U;
    uint32_t sum_rawL = 0U, sum_rawR = 0U;

    for (uint32_t i = 0U; i < ADC_BUF_LEN; i += 2U) {
        const uint32_t rawL = adc_buffer[i];
        const uint32_t rawR = adc_buffer[i + 1U];
        const uint32_t magL = u32_abs_diff(rawL, baseL);
        const uint32_t magR = u32_abs_diff(rawR, baseR);

        sum_rawL += rawL;
        sum_rawR += rawR;
        sum_magL += magL;
        sum_magR += magR;

        if (magL > max_magL) max_magL = magL;
        if (magR > max_magR) max_magR = magR;
    }

    const uint32_t frame_magL = sum_magL / samples_per_ch;
    const uint32_t frame_magR = sum_magR / samples_per_ch;
    adcAvgL = sum_rawL / samples_per_ch;
    adcAvgR = sum_rawR / samples_per_ch;

    ring_push_pair(sig_histL, sig_histR, SIG_WIN,
                   &sig_idx, &sig_count, &sig_sumL, &sig_sumR,
                   (uint16_t)frame_magL, (uint16_t)frame_magR);

    const uint32_t sig_avgL = (sig_count > 0U) ? (sig_sumL / sig_count) : 0U;
    const uint32_t sig_avgR = (sig_count > 0U) ? (sig_sumR / sig_count) : 0U;

    uint8_t freeze_noise = 0U;
    if (noise_count > 0U) {
        const uint32_t cur_noiseL = noise_sumL / noise_count;
        const uint32_t cur_noiseR = noise_sumR / noise_count;
        const uint32_t frame_snrL_q8 = q8_ratio(frame_magL, cur_noiseL);
        const uint32_t frame_snrR_q8 = q8_ratio(frame_magR, cur_noiseR);
        if (frame_snrL_q8 >= NOISE_FREEZE_Q8 || frame_snrR_q8 >= NOISE_FREEZE_Q8) {
            freeze_noise = 1U;
        }
    }

    if (!freeze_noise || noise_count < (NOISE_WIN / 4U)) {
        ring_push_pair(noise_histL, noise_histR, NOISE_WIN,
                       &noise_idx, &noise_count, &noise_sumL, &noise_sumR,
                       (uint16_t)frame_magL, (uint16_t)frame_magR);
    }

    noiseL = (noise_count > 0U) ? (noise_sumL / noise_count) : 1U;
    noiseR = (noise_count > 0U) ? (noise_sumR / noise_count) : 1U;
    noise_ready = (noise_count >= NOISE_WIN) ? 1U : 0U;
    snrL_q8 = q8_ratio(sig_avgL, noiseL);
    snrR_q8 = q8_ratio(sig_avgR, noiseR);

    peakL = max_magL;
    peakR = max_magR;
    lvlL = lvlL + (uint32_t)(((int32_t)sig_avgL - (int32_t)lvlL) / (int32_t)ALPHA_DIV);
    lvlR = lvlR + (uint32_t)(((int32_t)sig_avgR - (int32_t)lvlR) / (int32_t)ALPHA_DIV);
}

void app_loop(void)
{
    if (!is_calibrated) return;

    const uint32_t sigL = lvlL;
    const uint32_t sigR = lvlR;
    const uint32_t nL = noiseL;
    const uint32_t nR = noiseR;
    const uint32_t sL = snrL_q8;
    const uint32_t sR = snrR_q8;
    const uint8_t nReady = noise_ready;

    const uint32_t diff = u32_abs_diff(sigL, sigR);
    const uint32_t snr_diff = u32_abs_diff(sL, sR);
    const uint32_t nowm = HAL_GetTick();

    if ((int32_t)(nowm - motor_lock_until_ms) >= 0) {
        if (nReady &&
            (sigL >= SOUND_TH || sigR >= SOUND_TH) &&
            (diff >= DIFF_TH) &&
            (sL >= SNR_TH_Q8 || sR >= SNR_TH_Q8) &&
            (snr_diff >= SNR_DIFF_TH_Q8)) {

            const char newDir = (sL > sR) ? 'L' : 'R';
            const uint32_t now = HAL_GetTick();

            if (newDir != detectLR && (now - last_switch_ms >= SWITCH_HOLDOFF)) {
                detectLR = newDir;
                last_switch_ms = now;
            }
        }
    }

    static uint32_t last_dbg = 0U;
    if (nowm - last_dbg >= 200U) {
        /*
         * ADCavg_L / ADCavg_R are raw ADC averages for left/right channels.
         * Keep these fields in printf if you want both ADC channels in terminal.
         */
        printf("ADCavg_L:%4lu ADCavg_R:%4lu | RawPk_L:%4lu RawPk_R:%4lu | "
               "Sig_L:%4lu Sig_R:%4lu | Noise_L:%4lu Noise_R:%4lu | "
               "SNR_L:%2lu.%02lu SNR_R:%2lu.%02lu | Diff:%4lu | Lock:%4ld ms | Dir:%c\r\n",
               (unsigned long)adcAvgL, (unsigned long)adcAvgR,
               (unsigned long)peakL, (unsigned long)peakR,
               (unsigned long)sigL, (unsigned long)sigR,
               (unsigned long)nL, (unsigned long)nR,
               (unsigned long)(sL / 256U), (unsigned long)(((sL % 256U) * 100U) / 256U),
               (unsigned long)(sR / 256U), (unsigned long)(((sR % 256U) * 100U) / 256U),
               (unsigned long)diff,
               (int32_t)(motor_lock_until_ms - nowm) > 0 ? (int32_t)(motor_lock_until_ms - nowm) : 0,
               detectLR);
        last_dbg = nowm;
    }

    if (detectLR != last_dir) {
        last_dir = detectLR;

        if (detectLR == 'L' || detectLR == 'R') {
            if ((int32_t)(nowm - motor_lock_until_ms) < 0) {
                return;
            }

            if (motor_running) {
                pending_dir = detectLR;
            } else {
                pending_dir = '-';
                motor_running = 1U;
                motor_stop_ms = nowm + SERVO_RUN_MS;

                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                                      (detectLR == 'L') ? SERVO_LEFT_US : SERVO_RIGHT_US);

                motor_lock_until_ms = nowm + SERVO_RUN_MS + DIR_COOLDOWN_MS;
            }
        }
    }

    if (motor_running && (int32_t)(nowm - motor_stop_ms) >= 0) {
        motor_running = 0U;
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

        if (pending_dir == 'L' || pending_dir == 'R') {
            const char dir = pending_dir;
            pending_dir = '-';

            motor_running = 1U;
            motor_stop_ms = nowm + 20U + SERVO_RUN_MS;

            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                                  (dir == 'L') ? SERVO_LEFT_US : SERVO_RIGHT_US);
            motor_lock_until_ms = nowm + 20U + SERVO_RUN_MS + DIR_COOLDOWN_MS;
        }
    }
}
