#include "mic.h"

/* DMA 버퍼는 [L, R, L, R, ...] 형태로 채워짐 */
#define ADC_BUF_LEN 200U
static uint16_t adc_buffer[ADC_BUF_LEN];

/* 방향 판단 전 기본 게이트 */
#define SOUND_TH       400U /* 유효 소리로 볼 최소 신호 크기 */
#define ALPHA_DIV      4U   /* IIR 평활 계수: 클수록 더 부드럽고 느림 */
#define SWITCH_HOLDOFF 80U  /* 방향 토글 최소 간격(ms) */

/* 슬라이딩 윈도우 / SNR 게이트 */
#define SIG_WIN          6U   /* 최근 신호 추적용 짧은 창 */
#define NOISE_WIN       64U   /* 주변 소음 추적용 긴 창 */
#define SNR_TH_Q8      300U   /* 절대 SNR 임계값: 1.50배(Q8) */
#define NOISE_FREEZE_Q8 320U  /* 프레임 SNR이 1.25배 이상이면 노이즈 업데이트 중지 */
#define DIR_RATIO_TH_Q8 350U  /* 승자/패자 SNR 비율 최소 1.30배(Q8) */

/*
 * DMA ISR와 메인 루프가 공유하는 상태값.
 * 비동기 갱신되므로 volatile 필수.
 */
static volatile uint32_t baseL = 0U, baseR = 0U;
static volatile uint32_t lvlL = 0U, lvlR = 0U;
static volatile uint8_t is_calibrated = 0U;
static volatile uint32_t peakL = 0U, peakR = 0U;
static volatile uint32_t adcAvgL = 0U, adcAvgR = 0U;
static volatile uint32_t noiseL = 1U, noiseR = 1U;
static volatile uint32_t snrL_q8 = 0U, snrR_q8 = 0U;
static volatile uint8_t noise_ready = 0U;

/* 방향 결정 관련 상태 */
static char detectLR = '-';
static uint32_t last_switch_ms = 0U;
static volatile uint8_t gate_sound_dbg = 0U;
static volatile uint8_t gate_snr_dbg = 0U;
static volatile uint8_t gate_ratio_dbg = 0U;
static volatile uint32_t diff_dbg = 0U;

/*
 * 링버퍼 저장소 + 누적합.
 * 프레임마다 전체 합을 다시 구하지 않고 O(1) 이동평균 갱신.
 */
static uint16_t sig_histL[SIG_WIN];
static uint16_t sig_histR[SIG_WIN];
static uint16_t noise_histL[NOISE_WIN];
static uint16_t noise_histR[NOISE_WIN];
static uint16_t sig_idx = 0U, sig_count = 0U;
static uint16_t noise_idx = 0U, noise_count = 0U;
static uint32_t sig_sumL = 0U, sig_sumR = 0U;
static uint32_t noise_sumL = 0U, noise_sumR = 0U;

/* unsigned 절대 차이 유틸 */
static inline uint32_t u32_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* 비율 계산 유틸: (num / den)을 Q8 형식으로 반환 */
static inline uint32_t q8_ratio(uint32_t num, uint32_t den)
{
    if (den == 0U) den = 1U;
    return (num << 8) / den;
}

/*
 * 좌/우 샘플 한 쌍을 링버퍼에 푸시.
 * 버퍼가 가득 찬 경우 가장 오래된 값을 먼저 누적합에서 제거.
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

void mic_init(ADC_HandleTypeDef *hadc)
{
    detectLR = '-';
    last_switch_ms = HAL_GetTick();

    baseL = baseR = 0U;
    lvlL = lvlR = 0U;
    peakL = peakR = 0U;
    adcAvgL = adcAvgR = 0U;
    noiseL = noiseR = 1U;
    snrL_q8 = snrR_q8 = 0U;
    noise_ready = 0U;
    is_calibrated = 0U;

    gate_sound_dbg = gate_snr_dbg = gate_ratio_dbg = 0U;
    diff_dbg = 0U;

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

    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_buffer, ADC_BUF_LEN);
}

void mic_on_dma_complete(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    const uint32_t samples_per_ch = ADC_BUF_LEN / 2U;

    if (!is_calibrated) {
        /* 첫 프레임은 기준선 추정에만 사용 */
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

    /* 프레임 누적 변수 */
    uint32_t max_magL = 0U, max_magR = 0U;
    uint32_t sum_magL = 0U, sum_magR = 0U;
    uint32_t sum_rawL = 0U, sum_rawR = 0U;

    for (uint32_t i = 0U; i < ADC_BUF_LEN; i += 2U) {
        const uint32_t rawL = adc_buffer[i];
        const uint32_t rawR = adc_buffer[i + 1U];
        /* 기준선 대비 편차 크기: |raw - base| */
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

    /* UART 디버그용 raw ADC 프레임 평균 */
    adcAvgL = sum_rawL / samples_per_ch;
    adcAvgR = sum_rawR / samples_per_ch;

    /* 짧은 창: 현재 신호를 빠르게 반영 */
    ring_push_pair(sig_histL, sig_histR, SIG_WIN,
                   &sig_idx, &sig_count, &sig_sumL, &sig_sumR,
                   (uint16_t)frame_magL, (uint16_t)frame_magR);

    const uint32_t sig_avgL = (sig_count > 0U) ? (sig_sumL / sig_count) : 0U;
    const uint32_t sig_avgR = (sig_count > 0U) ? (sig_sumR / sig_count) : 0U;

    /* 이번 프레임으로 노이즈 바닥을 갱신할지 결정 */
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
     * 긴 창: 주변 소음을 천천히 추적.
     * 초기 워밍업 구간은 빠르게 채우기 위해 강제 업데이트 허용.
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

    /* 메인 루프 판단 안정화를 위한 신호 평활 */
    lvlL = lvlL + (uint32_t)(((int32_t)sig_avgL - (int32_t)lvlL) / (int32_t)ALPHA_DIV);
    lvlR = lvlR + (uint32_t)(((int32_t)sig_avgR - (int32_t)lvlR) / (int32_t)ALPHA_DIV);
}

char mic_process(uint32_t now_ms, uint32_t motor_lock_until_ms)
{
    if (!is_calibrated) {
        gate_sound_dbg = 0U;
        gate_snr_dbg = 0U;
        gate_ratio_dbg = 0U;
        diff_dbg = 0U;
        return detectLR;
    }

    /* 이번 루프에서 사용할 ISR 값 스냅샷 */
    const uint32_t sigL = lvlL;
    const uint32_t sigR = lvlR;
    const uint32_t sL = snrL_q8;
    const uint32_t sR = snrR_q8;
    const uint8_t nReady = noise_ready;

    diff_dbg = u32_abs_diff(sigL, sigR);

    /* 디버그/튜닝용 게이트 상태 */
    const uint8_t gate_sound = ((sigL >= SOUND_TH || sigR >= SOUND_TH) ? 1U : 0U);
    const uint8_t gate_snr = ((sL >= SNR_TH_Q8 || sR >= SNR_TH_Q8) ? 1U : 0U);
    const uint32_t win = (sL > sR) ? sL : sR;
    const uint32_t lose = (sL > sR) ? sR : sL;
    const uint8_t gate_ratio = (((win << 8) >= (lose * DIR_RATIO_TH_Q8)) ? 1U : 0U);

    gate_sound_dbg = gate_sound;
    gate_snr_dbg = gate_snr;
    gate_ratio_dbg = gate_ratio;

    /* 모터 락이 해제된 경우에만 방향 업데이트 */
    if ((int32_t)(now_ms - motor_lock_until_ms) >= 0) {
        if (nReady && gate_sound && gate_snr && gate_ratio) {
            const char newDir = (sL > sR) ? 'L' : 'R';
            if (newDir != detectLR && (now_ms - last_switch_ms >= SWITCH_HOLDOFF)) {
                detectLR = newDir;
                last_switch_ms = now_ms;
            }
        }
    }

    return detectLR;
}

uint8_t mic_is_calibrated(void)
{
    return is_calibrated;
}

void mic_get_debug(mic_debug_t *out)
{
    if (out == NULL) return;

    out->adc_avg_l = adcAvgL;
    out->adc_avg_r = adcAvgR;
    out->peak_l = peakL;
    out->peak_r = peakR;
    out->sig_l = lvlL;
    out->sig_r = lvlR;
    out->noise_l = noiseL;
    out->noise_r = noiseR;
    out->snr_l_q8 = snrL_q8;
    out->snr_r_q8 = snrR_q8;
    out->diff = diff_dbg;
    out->gate_sound = gate_sound_dbg;
    out->gate_snr = gate_snr_dbg;
    out->gate_ratio = gate_ratio_dbg;
    out->noise_ready = noise_ready;
    out->detect_dir = detectLR;
}
