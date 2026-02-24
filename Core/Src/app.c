#include "app.h"
#include "mic.h"
#include "motor_ctrl.h"
#include <stdio.h>

/* main.c에서 생성된 HAL 핸들 */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
//
/*
 * 앱 오케스트레이션:
 * - mic 모듈: DMA 기반 마이크 신호 처리 + 방향 감지
 * - motor_ctrl 모듈: 서보 상태머신 + 락/쿨다운 처리
 * - app.c: 두 모듈 연계 및 디버그 출력
 */
void app_init(void)
{
    motor_ctrl_init(&htim3);
    mic_init(&hadc1);
    printf("Starting DMA Audio System...\r\n");
}

/* ADC DMA 완료 콜백은 mic 모듈에 위임 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mic_on_dma_complete(hadc);
}

void app_loop(void)
{
    if (!mic_is_calibrated()) return;

    const uint32_t nowm = HAL_GetTick();
    const uint32_t lock_until_ms = motor_ctrl_get_lock_until_ms();

    /* 모터 락 상태를 고려해 현재 방향 갱신 */
    const char detect_dir = mic_process(nowm, lock_until_ms);

    /* UART 디버그 출력 */
    static uint32_t last_dbg = 0U;
    if (nowm - last_dbg >= 200U) {
        mic_debug_t dbg;
        mic_get_debug(&dbg);

        printf("ADCavg_L:%4lu ADCavg_R:%4lu | RawPk_L:%4lu RawPk_R:%4lu | "
               "Sig_L:%4lu Sig_R:%4lu | Noise_L:%4lu Noise_R:%4lu | "
               "SNR_L:%2lu.%02lu SNR_R:%2lu.%02lu | Diff:%4lu | Gate[S:%u N:%u R:%u] | Lock:%4ld ms | Dir:%c\r\n",
               (unsigned long)dbg.adc_avg_l, (unsigned long)dbg.adc_avg_r,
               (unsigned long)dbg.peak_l, (unsigned long)dbg.peak_r,
               (unsigned long)dbg.sig_l, (unsigned long)dbg.sig_r,
               (unsigned long)dbg.noise_l, (unsigned long)dbg.noise_r,
               (unsigned long)(dbg.snr_l_q8 / 256U), (unsigned long)(((dbg.snr_l_q8 % 256U) * 100U) / 256U),
               (unsigned long)(dbg.snr_r_q8 / 256U), (unsigned long)(((dbg.snr_r_q8 % 256U) * 100U) / 256U),
               (unsigned long)dbg.diff,
               (unsigned int)dbg.gate_sound, (unsigned int)dbg.gate_snr, (unsigned int)dbg.gate_ratio,
               (int32_t)(lock_until_ms - nowm) > 0 ? (int32_t)(lock_until_ms - nowm) : 0,
               detect_dir);
        last_dbg = nowm;
    }

    /* 감지 방향으로 서보 상태머신 업데이트 */
    motor_ctrl_process(nowm, detect_dir);
}
