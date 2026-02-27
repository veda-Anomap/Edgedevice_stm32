#include "aht10.h"

/* AHT10/AHT20 I2C 주소 (7-bit 0x38, HAL에서는 1비트 좌시프트 주소 사용) */
#define AHT10_ADDR            (0x38U << 1)
//
/* 명령어 */
#define AHT10_CMD_INIT_0      0xE1U
#define AHT10_CMD_INIT_1      0x08U
#define AHT10_CMD_INIT_2      0x00U
#define AHT10_CMD_MEAS_0      0xACU
#define AHT10_CMD_MEAS_1      0x33U
#define AHT10_CMD_MEAS_2      0x00U

/* 시간 관련 상수 */
#define AHT10_CONV_TIME_MS    80U
#define AHT10_PERIOD_DEFAULT  2000U
#define AHT10_PERIOD_MIN_MS   1000U
#define AHT10_FILTER_WIN      10U
#define AHT10_FULL_SCALE_20B  1048576ULL

typedef enum {
    AHT10_STATE_IDLE = 0,
    AHT10_STATE_WAIT_CONV
} aht10_state_t;

static I2C_HandleTypeDef *s_hi2c = NULL;
static aht10_state_t s_state = AHT10_STATE_IDLE;
static uint32_t s_period_ms = AHT10_PERIOD_DEFAULT;
static uint32_t s_next_measure_ms = 0U;
static uint32_t s_conv_due_ms = 0U;

static int32_t s_temp_c_x100 = 0;
static uint32_t s_humi_rh_x100 = 0U;
static uint32_t s_last_update_ms = 0U;
static uint32_t s_error_count = 0U;
static uint8_t s_valid = 0U;
static uint8_t s_busy = 0U;

/* N=10 FIFO(원형버퍼) 이동평균 */
static int32_t s_temp_hist[AHT10_FILTER_WIN];
static uint32_t s_humi_hist[AHT10_FILTER_WIN];
static uint8_t s_hist_idx = 0U;
static uint8_t s_hist_count = 0U;
static int64_t s_temp_sum = 0;
static uint64_t s_humi_sum = 0ULL;

static void aht10_filter_push(int32_t temp_c_x100, uint32_t humi_rh_x100)
{
    if (s_hist_count == AHT10_FILTER_WIN) {
        s_temp_sum -= s_temp_hist[s_hist_idx];
        s_humi_sum -= s_humi_hist[s_hist_idx];
    } else {
        s_hist_count++;
    }

    s_temp_hist[s_hist_idx] = temp_c_x100;
    s_humi_hist[s_hist_idx] = humi_rh_x100;
    s_temp_sum += temp_c_x100;
    s_humi_sum += humi_rh_x100;

    s_hist_idx = (uint8_t)((s_hist_idx + 1U) % AHT10_FILTER_WIN);

    s_temp_c_x100 = (int32_t)(s_temp_sum / s_hist_count);
    s_humi_rh_x100 = (uint32_t)(s_humi_sum / s_hist_count);
}
//
static void aht10_send_init_once(void)
{
    uint8_t cmd[3] = { AHT10_CMD_INIT_0, AHT10_CMD_INIT_1, AHT10_CMD_INIT_2 };
    (void)HAL_I2C_Master_Transmit(s_hi2c, AHT10_ADDR, cmd, sizeof(cmd), 100);
}

void aht10_init(I2C_HandleTypeDef *hi2c, uint32_t period_ms)
{
    s_hi2c = hi2c;
    s_state = AHT10_STATE_IDLE;
    /* 자가발열 방지: 측정 주기는 최소 1초 이상으로 강제 */
    if (period_ms == 0U) {
        s_period_ms = AHT10_PERIOD_DEFAULT;
    } else if (period_ms < AHT10_PERIOD_MIN_MS) {
        s_period_ms = AHT10_PERIOD_MIN_MS;
    } else {
        s_period_ms = period_ms;
    }
    s_next_measure_ms = 0U;
    s_conv_due_ms = 0U;

    s_temp_c_x100 = 0;
    s_humi_rh_x100 = 0U;
    s_last_update_ms = 0U;
    s_error_count = 0U;
    s_valid = 0U;
    s_busy = 0U;
    s_hist_idx = 0U;
    s_hist_count = 0U;
    s_temp_sum = 0;
    s_humi_sum = 0ULL;

    for (uint32_t i = 0U; i < AHT10_FILTER_WIN; i++) {
        s_temp_hist[i] = 0;
        s_humi_hist[i] = 0U;
    }

    if (s_hi2c != NULL) {
        aht10_send_init_once();
    }
}

void aht10_process(uint32_t now_ms)
{
    if (s_hi2c == NULL) return;

    if (s_state == AHT10_STATE_IDLE) {
        if ((int32_t)(now_ms - s_next_measure_ms) >= 0) {
            uint8_t cmd[3] = { AHT10_CMD_MEAS_0, AHT10_CMD_MEAS_1, AHT10_CMD_MEAS_2 };
            if (HAL_I2C_Master_Transmit(s_hi2c, AHT10_ADDR, cmd, sizeof(cmd), 100) == HAL_OK) {
                s_busy = 1U;
                s_state = AHT10_STATE_WAIT_CONV;
                s_conv_due_ms = now_ms + AHT10_CONV_TIME_MS;
            } else {
                s_error_count++;
                s_next_measure_ms = now_ms + s_period_ms;
            }
        }
        return;
    }

    /* WAIT_CONV: 변환 완료 전에는 읽지 않음(최소 80ms 대기) */
    if ((int32_t)(now_ms - s_conv_due_ms) < 0) return;

    uint8_t rx[6] = {0};
    if (HAL_I2C_Master_Receive(s_hi2c, AHT10_ADDR, rx, sizeof(rx), 100) != HAL_OK) {
        s_error_count++;
        s_busy = 0U;
        s_state = AHT10_STATE_IDLE;
        s_next_measure_ms = now_ms + s_period_ms;
        return;
    }

    /* 상태 바이트 bit7(BUSY)가 1이면 아직 변환 중이므로 짧게 재시도 */
    if ((rx[0] & 0x80U) != 0U) {
        s_conv_due_ms = now_ms + 10U;
        return;
    }

    /* 20비트 습도/온도 원시값 추출 */
    uint32_t raw_h = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | ((uint32_t)rx[3] >> 4);
    uint32_t raw_t = (((uint32_t)rx[3] & 0x0FU) << 16) | ((uint32_t)rx[4] << 8) | (uint32_t)rx[5];

    /* 습도[%RH*100] = raw_h * 10000 / 2^20 */
    const uint32_t humi_raw_x100 = (uint32_t)(((uint64_t)raw_h * 10000ULL) / AHT10_FULL_SCALE_20B);

    /* 온도[C*100] = (raw_t * 20000 / 2^20) - 5000 */
    const int32_t temp_raw_x100 = (int32_t)(((uint64_t)raw_t * 20000ULL) / AHT10_FULL_SCALE_20B) - 5000;

    /* 단일 샘플이 아니라 최근 N=10 평균값을 최종 출력값으로 사용 */
    aht10_filter_push(temp_raw_x100, humi_raw_x100);

    s_valid = 1U;
    s_busy = 0U;
    s_last_update_ms = now_ms;
    s_state = AHT10_STATE_IDLE;
    s_next_measure_ms = now_ms + s_period_ms;
}

void aht10_get_data(aht10_data_t *out)
{
    if (out == NULL) return;

    out->temperature_c_x100 = s_temp_c_x100;
    out->humidity_rh_x100 = s_humi_rh_x100;
    out->valid = s_valid;
    out->busy = s_busy;
    out->period_ms = s_period_ms;
    out->last_update_ms = s_last_update_ms;
    out->error_count = s_error_count;
}
