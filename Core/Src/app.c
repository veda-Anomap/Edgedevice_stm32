#include "app.h"
#include "mic.h"
#include "motor_ctrl.h"
#include "aht10.h"
#include <stdio.h>

/* main.c 에서 생성된 HAL 핸들 */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;

/*
 * 앱 오케스트레이션 레이어
 * - mic 모듈: DMA 기반 음향 처리/방향 판정
 * - motor_ctrl 모듈: 서보 상태머신 제어
 * - aht10 모듈: 온습도 비차단 주기 측정
 */
void app_init(void)
{
    motor_ctrl_init(&htim3);
    mic_init(&hadc1);

    /* 자가발열 방지를 위해 측정 주기를 2초로 설정 */
    aht10_init(&hi2c1, 2000U);

    printf("Starting DMA Audio System...\r\n");
}

/* ADC DMA 완료 콜백은 mic 모듈로 위임 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mic_on_dma_complete(hadc);
}

void app_loop(void)
{
    const uint32_t nowm = HAL_GetTick();
    const uint32_t lock_until_ms = motor_ctrl_get_lock_until_ms();

    /*
     * 온습도 상태머신 1스텝 진행
     * - 측정 명령 후 최소 80ms가 지나야 읽기를 수행
     * - while(1)에서 계속 호출해도 내부 주기(2초)로만 실제 측정
     */
    aht10_process(nowm);

    if (!mic_is_calibrated()) {
        return;
    }

    /* 모터 락 상태를 반영한 방향 감지 */
    const char detect_dir = mic_process(nowm, lock_until_ms);

    /* UART 디버그 출력 */
    static uint32_t last_dbg = 0U;
    if (nowm - last_dbg >= 200U) {
        mic_debug_t dbg;
        aht10_data_t th;
        mic_get_debug(&dbg);
        aht10_get_data(&th);

        const int32_t temp_abs = (th.temperature_c_x100 < 0) ? -th.temperature_c_x100 : th.temperature_c_x100;
        const char temp_sign = (th.temperature_c_x100 < 0) ? '-' : '+';

        printf("ADCavg_L:%4lu ADCavg_R:%4lu | RawPk_L:%4lu RawPk_R:%4lu | "
               "Sig_L:%4lu Sig_R:%4lu | Noise_L:%4lu Noise_R:%4lu | "
               "SNR_L:%2lu.%02lu SNR_R:%2lu.%02lu | Diff:%4lu | Gate[S:%u N:%u R:%u] | "
               "TH[V:%u B:%u E:%lu] T:%c%ld.%02ldC H:%lu.%02lu%% | Lock:%4ld ms | Dir:%c\r\n",
               (unsigned long)dbg.adc_avg_l, (unsigned long)dbg.adc_avg_r,
               (unsigned long)dbg.peak_l, (unsigned long)dbg.peak_r,
               (unsigned long)dbg.sig_l, (unsigned long)dbg.sig_r,
               (unsigned long)dbg.noise_l, (unsigned long)dbg.noise_r,
               (unsigned long)(dbg.snr_l_q8 / 256U), (unsigned long)(((dbg.snr_l_q8 % 256U) * 100U) / 256U),
               (unsigned long)(dbg.snr_r_q8 / 256U), (unsigned long)(((dbg.snr_r_q8 % 256U) * 100U) / 256U),
               (unsigned long)dbg.diff,
               (unsigned int)dbg.gate_sound, (unsigned int)dbg.gate_snr, (unsigned int)dbg.gate_ratio,
               (unsigned int)th.valid, (unsigned int)th.busy, (unsigned long)th.error_count,
               temp_sign, (long)(temp_abs / 100), (long)(temp_abs % 100),
               (unsigned long)(th.humidity_rh_x100 / 100U), (unsigned long)(th.humidity_rh_x100 % 100U),
               (int32_t)(lock_until_ms - nowm) > 0 ? (int32_t)(lock_until_ms - nowm) : 0,
               detect_dir);
        last_dbg = nowm;
    }

    /* 감지 방향으로 서보 상태머신 업데이트 */
    motor_ctrl_process(nowm, detect_dir);
}
