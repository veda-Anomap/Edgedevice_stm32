#include "app.h"
#include <stdint.h>
#include <stdio.h>

/* main.c에서 생성된 HAL 핸들 */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* DMA 버퍼는 [L, R, L, R, ...] 형태로 채워짐 */
#define ADC_BUF_LEN 200U
uint16_t adc_buffer[ADC_BUF_LEN];

/* 방향 판단 전 기본 게이트 */
#define SOUND_TH       400U /* 유효 소리로 볼 최소 신호 크기 */
#define ALPHA_DIV      4U   /* IIR 평활 계수: 클수록 더 부드럽고 느림 */
#define SWITCH_HOLDOFF 80U  /* 방향 토글 최소 간격(ms) */

/* 슬라이딩 윈도우 / SNR 게이트 */
#define SIG_WIN          6U   /* 최근 신호 추적용 짧은 창 */
#define NOISE_WIN       64U   /* 주변 소음 추적용 긴 창 */
#define SNR_TH_Q8      384U   /* 절대 SNR 임계값: 1.50배(Q8) */
#define NOISE_FREEZE_Q8 320U  /* 프레임 SNR이 1.25배 이상이면 노이즈 업데이트 중지 */
#define DIR_RATIO_TH_Q8 333U  /* 승자/패자 SNR 비율 최소 1.30배(Q8) */

/* 서보 구동 타이밍 */
#define SERVO_LEFT_US    1200U /* 좌회전 명령 PWM 펄스폭 */
#define SERVO_RIGHT_US   1820U /* 우회전 명령 PWM 펄스폭 */
#define SERVO_CENTER_US  1520U /* 중앙 PWM 펄스폭 */
#define SERVO_RUN_MS      200U /* 명령당 서보 구동 시간(ms) */
#define DIR_COOLDOWN_MS  1500U /* 명령 후 재트리거 방지 락(ms) */

/*
 * DMA ISR와 메인 루프가 공유하는 상태값.
 * 비동기 갱신되므로 volatile 필수.
 */
static volatile uint32_t baseL = 0U, baseR = 0U;     /* 채널별 기준선(DC 오프셋) */
static volatile uint32_t lvlL = 0U, lvlR = 0U;       /* 메인 루프가 쓰는 평활 신호 레벨 */
static volatile uint8_t is_calibrated = 0U;          /* 기준선 계산 완료 플래그 */
static volatile uint32_t peakL = 0U, peakR = 0U;     /* 프레임 최대 편차(피크) */
static volatile uint32_t adcAvgL = 0U, adcAvgR = 0U; /* 프레임 raw ADC 평균 */
static volatile uint32_t noiseL = 1U, noiseR = 1U;   /* 채널별 노이즈 바닥 추정 */
static volatile uint32_t snrL_q8 = 0U, snrR_q8 = 0U; /* Q8 고정소수점 SNR */
static volatile uint8_t noise_ready = 0U;            /* 노이즈 창 워밍업 완료 여부 */

/* 방향/서보 상태머신 */
static char detectLR = '-';               /* 최신 감지 방향: 'L', 'R', '-' */
static uint32_t last_switch_ms = 0U;      /* 마지막 방향 변경 시각 */
static uint32_t motor_lock_until_ms = 0U; /* 이 시각 전에는 신규 명령 무시 */
static char last_dir = '-';               /* 이미 소비된 마지막 방향 */
static uint8_t motor_running = 0U;        /* 서보 명령 실행 중 여부 */
static uint32_t motor_stop_ms = 0U;       /* 현재 명령 종료 시각 */
static char pending_dir = '-';            /* 서보 바쁠 때 대기시킬 방향 */

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

/*
 * 앱 상태를 초기화하고 ADC DMA 스트림 시작.
 * 주변장치 초기화 완료 후 main에서 1회 호출.
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
 * ADC DMA 완료 콜백(프레임 단위 처리):
 * 1) 최초 1회 기준선 계산
 * 2) 프레임 raw 평균/편차 평균/피크 계산
 * 3) 짧은 신호 창 + 긴 노이즈 창 갱신
 * 4) 메인 루프에서 쓸 신호/노이즈/SNR 상태 갱신
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
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

/*
 * 메인 루프 처리:
 * - ISR 값 스냅샷 획득
 * - 레벨/SNR/비율 게이트 적용
 * - 방향 상태 및 서보 상태머신 갱신
 */
void app_loop(void)
{
    if (!is_calibrated) return;

    /* 이번 루프에서 사용할 ISR 값 스냅샷 */
    const uint32_t sigL = lvlL;
    const uint32_t sigR = lvlR;
    const uint32_t nL = noiseL;
    const uint32_t nR = noiseR;
    const uint32_t sL = snrL_q8;
    const uint32_t sR = snrR_q8;
    const uint8_t nReady = noise_ready;

    const uint32_t diff = u32_abs_diff(sigL, sigR);
    const uint32_t nowm = HAL_GetTick();

    /* 디버그/튜닝용 게이트 상태 */
    const uint8_t gate_sound = ((sigL >= SOUND_TH || sigR >= SOUND_TH) ? 1U : 0U);
    const uint8_t gate_snr = ((sL >= SNR_TH_Q8 || sR >= SNR_TH_Q8) ? 1U : 0U);
    const uint32_t win = (sL > sR) ? sL : sR;
    const uint32_t lose = (sL > sR) ? sR : sL;
    const uint8_t gate_ratio = (((win << 8) >= (lose * DIR_RATIO_TH_Q8)) ? 1U : 0U);

    /* 모터 락이 해제된 경우에만 방향 업데이트 */
    if ((int32_t)(nowm - motor_lock_until_ms) >= 0) {
        if (nReady && gate_sound && gate_snr && gate_ratio) {
            const char newDir = (sL > sR) ? 'L' : 'R';
            const uint32_t now = HAL_GetTick();

            if (newDir != detectLR && (now - last_switch_ms >= SWITCH_HOLDOFF)) {
                detectLR = newDir;
                last_switch_ms = now;
            }
        }
    }

    /* UART 디버그 출력 */
    static uint32_t last_dbg = 0U;
    if (nowm - last_dbg >= 200U) {
        /*
         * ADCavg_L / ADCavg_R:
         * - DMA 버퍼에서 계산한 raw ADC 프레임 평균
         * - 좌/우 입력 밸런스 확인에 유용
         */
        printf("ADCavg_L:%4lu ADCavg_R:%4lu | RawPk_L:%4lu RawPk_R:%4lu | "
               "Sig_L:%4lu Sig_R:%4lu | Noise_L:%4lu Noise_R:%4lu | "
               "SNR_L:%2lu.%02lu SNR_R:%2lu.%02lu | Diff:%4lu | Gate[S:%u N:%u R:%u] | Lock:%4ld ms | Dir:%c\r\n",
               (unsigned long)adcAvgL, (unsigned long)adcAvgR,
               (unsigned long)peakL, (unsigned long)peakR,
               (unsigned long)sigL, (unsigned long)sigR,
               (unsigned long)nL, (unsigned long)nR,
               (unsigned long)(sL / 256U), (unsigned long)(((sL % 256U) * 100U) / 256U),
               (unsigned long)(sR / 256U), (unsigned long)(((sR % 256U) * 100U) / 256U),
               (unsigned long)diff,
               (unsigned int)gate_sound, (unsigned int)gate_snr, (unsigned int)gate_ratio,
               (int32_t)(motor_lock_until_ms - nowm) > 0 ? (int32_t)(motor_lock_until_ms - nowm) : 0,
               detectLR);
        last_dbg = nowm;
    }

    /* 방향 변화 감지 시 서보 명령 스케줄링 */
    if (detectLR != last_dir) {
        last_dir = detectLR;

        if (detectLR == 'L' || detectLR == 'R') {
            if ((int32_t)(nowm - motor_lock_until_ms) < 0) {
                return;
            }

            if (motor_running) {
                /* 서보가 바쁘면 최신 방향만 대기열로 유지 */
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

    /* 현재 명령 종료 후 필요 시 pending 명령 1회 실행 */
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
