#include "app.h"
#include <stdint.h>
#include <stdio.h>

/* HAL handles created in main.c */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* DMA writes interleaved samples: [L, R, L, R, ...] */
#define ADC_BUF_LEN 200U
uint16_t adc_buffer[ADC_BUF_LEN];

/* Base signal gates (applied before direction decision) */
#define SOUND_TH       400U /* Minimum signal magnitude to treat as valid sound */
#define DIFF_TH        220U /* Minimum left/right signal difference */
#define ALPHA_DIV      4U   /* IIR smoothing factor: larger = smoother/slower */
#define SWITCH_HOLDOFF 80U  /* Min interval between direction toggles (ms) */

/* Sliding-window / SNR gates */
#define SIG_WIN          6U   /* Short window for current signal level */
#define NOISE_WIN       64U   /* Long window for ambient noise floor */
#define SNR_TH_Q8      384U   /* Absolute SNR threshold: 1.50x in Q8 */
#define SNR_DIFF_TH_Q8 102U   /* Left-right SNR gap threshold: 0.40x in Q8 */
#define NOISE_FREEZE_Q8 320U  /* Freeze noise update when frame SNR >= 1.25x */
#define DIR_RATIO_TH_Q8 333U  /* Winner/loser SNR ratio >= 1.30x in Q8 */
#define PEAK_DIFF_TH    120U  /* Extra peak-difference gate */

/* Servo command timings */
#define SERVO_LEFT_US    1200U /* PWM pulse for left */
#define SERVO_RIGHT_US   1820U /* PWM pulse for right */
#define SERVO_CENTER_US  1520U /* PWM pulse for center */
#define SERVO_RUN_MS      200U /* Servo active duration per command */
#define DIR_COOLDOWN_MS  1500U /* Lockout after command to avoid chatter */

/*
 * Shared state between DMA ISR and main loop.
 * volatile is required because values are updated asynchronously.
 */
static volatile uint32_t baseL = 0U, baseR = 0U;     /* Per-channel baseline (DC offset) */
static volatile uint32_t lvlL = 0U, lvlR = 0U;       /* Smoothed signal level used by loop */
static volatile uint8_t is_calibrated = 0U;          /* Baseline ready flag */
static volatile uint32_t peakL = 0U, peakR = 0U;     /* Per-frame peak magnitude */
static volatile uint32_t adcAvgL = 0U, adcAvgR = 0U; /* Per-frame raw ADC averages */
static volatile uint32_t noiseL = 1U, noiseR = 1U;   /* Noise floor estimate */
static volatile uint32_t snrL_q8 = 0U, snrR_q8 = 0U; /* SNR in Q8 fixed-point format */
static volatile uint8_t noise_ready = 0U;            /* True when noise window is warmed up */

/* Direction and servo state machine */
static char detectLR = '-';               /* Latest detected direction: 'L', 'R', '-' */
static uint32_t last_switch_ms = 0U;      /* Timestamp of last direction change */
static uint32_t motor_lock_until_ms = 0U; /* Ignore new commands before this time */
static char last_dir = '-';               /* Last direction already consumed by motor */
static uint8_t motor_running = 0U;        /* Servo currently executing a command */
static uint32_t motor_stop_ms = 0U;       /* Stop time for current servo command */
static char pending_dir = '-';            /* Queued direction while servo is busy */

/*
 * Ring-buffer storage + running sums.
 * This gives O(1) moving-average updates (no full-window loops each frame).
 */
static uint16_t sig_histL[SIG_WIN];
static uint16_t sig_histR[SIG_WIN];
static uint16_t noise_histL[NOISE_WIN];
static uint16_t noise_histR[NOISE_WIN];
static uint16_t sig_idx = 0U, sig_count = 0U;
static uint16_t noise_idx = 0U, noise_count = 0U;
static uint32_t sig_sumL = 0U, sig_sumR = 0U;
static uint32_t noise_sumL = 0U, noise_sumR = 0U;

/* Unsigned absolute difference helper */
static inline uint32_t u32_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* Ratio helper: returns (num / den) in Q8 fixed-point */
static inline uint32_t q8_ratio(uint32_t num, uint32_t den)
{
    if (den == 0U) den = 1U;
    return (num << 8) / den;
}

/*
 * Push one left/right sample pair into a ring buffer.
 * If full, oldest sample is removed from running sums first.
 */
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

/*
 * Initialize app state and start ADC DMA stream.
 * Called once from main after peripheral init.
 */
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

/*
 * ADC DMA complete callback (frame processing):
 * 1) Build baseline once (first frame).
 * 2) Compute frame-level raw average, magnitude average, and peak.
 * 3) Update short signal window and long noise window.
 * 4) Publish smoothed signal/noise/SNR values for app_loop().
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    const uint32_t samples_per_ch = ADC_BUF_LEN / 2U;

    if (!is_calibrated) {
        /* First frame is used only to estimate channel baselines. */
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

    /* Frame accumulators */
    uint32_t max_magL = 0U, max_magR = 0U;
    uint32_t sum_magL = 0U, sum_magR = 0U;
    uint32_t sum_rawL = 0U, sum_rawR = 0U;

    for (uint32_t i = 0U; i < ADC_BUF_LEN; i += 2U) {
        const uint32_t rawL = adc_buffer[i];
        const uint32_t rawR = adc_buffer[i + 1U];
        /* Magnitude relative to baseline: |raw - base| */
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

    /* Raw ADC monitor values (for UART debug output). */
    adcAvgL = sum_rawL / samples_per_ch;
    adcAvgR = sum_rawR / samples_per_ch;

    /* Short window tracks current signal quickly. */
    ring_push_pair(sig_histL, sig_histR, SIG_WIN,
                   &sig_idx, &sig_count, &sig_sumL, &sig_sumR,
                   (uint16_t)frame_magL, (uint16_t)frame_magR);

    const uint32_t sig_avgL = (sig_count > 0U) ? (sig_sumL / sig_count) : 0U;
    const uint32_t sig_avgR = (sig_count > 0U) ? (sig_sumR / sig_count) : 0U;

    /* Decide whether this frame should update noise floor. */
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

    /*
     * Long window tracks ambient noise slowly.
     * During early warm-up, force updates so noise window fills.
     */
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

    /* Smooth signal levels for stable decision in main loop. */
    lvlL = lvlL + (uint32_t)(((int32_t)sig_avgL - (int32_t)lvlL) / (int32_t)ALPHA_DIV);
    lvlR = lvlR + (uint32_t)(((int32_t)sig_avgR - (int32_t)lvlR) / (int32_t)ALPHA_DIV);
}

/*
 * Main-loop decision and servo control:
 * - Read ISR outputs once (snapshot).
 * - Apply level/SNR/ratio gates.
 * - Update direction and servo state machine.
 */
void app_loop(void)
{
    if (!is_calibrated) return;

    /* Snapshot of ISR-updated values for this loop iteration. */
    const uint32_t sigL = lvlL;
    const uint32_t sigR = lvlR;
    const uint32_t nL = noiseL;
    const uint32_t nR = noiseR;
    const uint32_t sL = snrL_q8;
    const uint32_t sR = snrR_q8;
    const uint32_t pkL = peakL;
    const uint32_t pkR = peakR;
    const uint8_t nReady = noise_ready;

    const uint32_t diff = u32_abs_diff(sigL, sigR);
    const uint32_t snr_diff = u32_abs_diff(sL, sR);
    const uint32_t peak_diff = u32_abs_diff(pkL, pkR);
    const uint32_t nowm = HAL_GetTick();

    /* Direction update is blocked while motor command lock is active. */
    if ((int32_t)(nowm - motor_lock_until_ms) >= 0) {
        if (nReady &&
            (sigL >= SOUND_TH || sigR >= SOUND_TH) &&
            (diff >= DIFF_TH) &&
            (sL >= SNR_TH_Q8 || sR >= SNR_TH_Q8) &&
            (snr_diff >= SNR_DIFF_TH_Q8) &&
            (peak_diff >= PEAK_DIFF_TH)) {

            const uint32_t win = (sL > sR) ? sL : sR;
            const uint32_t lose = (sL > sR) ? sR : sL;
            /* Final gate: winner SNR must clearly dominate loser SNR. */
            const uint8_t ratio_ok = ((win << 8) >= (lose * DIR_RATIO_TH_Q8));
            if (ratio_ok) {
                const char newDir = (sL > sR) ? 'L' : 'R';
                const uint32_t now = HAL_GetTick();

                if (newDir != detectLR && (now - last_switch_ms >= SWITCH_HOLDOFF)) {
                    detectLR = newDir;
                    last_switch_ms = now;
                }
            }
        }
    }

    /* UART debug print */
    static uint32_t last_dbg = 0U;
    if (nowm - last_dbg >= 200U) {
        /*
         * ADCavg_L / ADCavg_R:
         * - raw ADC frame averages from DMA buffer
         * - useful for checking left/right input balance
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

    /* Direction change -> servo command scheduling */
    if (detectLR != last_dir) {
        last_dir = detectLR;

        if (detectLR == 'L' || detectLR == 'R') {
            if ((int32_t)(nowm - motor_lock_until_ms) < 0) {
                return;
            }

            if (motor_running) {
                /* Keep only the newest pending direction while servo is busy. */
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

    /* Stop active servo pulse window and optionally run one pending command. */
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
