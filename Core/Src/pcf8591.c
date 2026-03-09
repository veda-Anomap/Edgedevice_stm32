#include "pcf8591.h"

/* PCF8591 7-bit address (A2/A1/A0 = 0) */
#define PCF8591_ADDR            (0x48U << 1)
/* Control byte: analog input enable + channel 0 */
#define PCF8591_CTRL_CH0        0x40U

#define PCF8591_PERIOD_DEFAULT  200U
#define PCF8591_PERIOD_MIN_MS   50U

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint32_t s_period_ms = PCF8591_PERIOD_DEFAULT;
static uint32_t s_next_ms = 0U;

static uint32_t s_light_raw = 0U;
static uint32_t s_last_update_ms = 0U;
static uint32_t s_error_count = 0U;
static uint8_t s_valid = 0U;

void pcf8591_init(I2C_HandleTypeDef *hi2c, uint32_t period_ms)
{
    s_hi2c = hi2c;
    if (period_ms == 0U) {
        s_period_ms = PCF8591_PERIOD_DEFAULT;
    } else if (period_ms < PCF8591_PERIOD_MIN_MS) {
        s_period_ms = PCF8591_PERIOD_MIN_MS;
    } else {
        s_period_ms = period_ms;
    }
    s_next_ms = HAL_GetTick();
    s_light_raw = 0U;
    s_last_update_ms = 0U;
    s_error_count = 0U;
    s_valid = 0U;
}

void pcf8591_process(uint32_t now_ms)
{
    uint8_t ctrl = PCF8591_CTRL_CH0;
    uint8_t rx[2] = {0};

    if (s_hi2c == NULL) return;
    if ((int32_t)(now_ms - s_next_ms) < 0) return;
    s_next_ms = now_ms + s_period_ms;

    if (HAL_I2C_Master_Transmit(s_hi2c, PCF8591_ADDR, &ctrl, 1U, 100) != HAL_OK) {
        s_error_count++;
        return;
    }

    /* First byte is dummy, second byte is current conversion result */
    if (HAL_I2C_Master_Receive(s_hi2c, PCF8591_ADDR, rx, sizeof(rx), 100) != HAL_OK) {
        s_error_count++;
        return;
    }

    s_light_raw = (uint32_t)rx[1];
    s_last_update_ms = now_ms;
    s_valid = 1U;
}

void pcf8591_get_data(pcf8591_data_t *out)
{
    if (out == NULL) return;
    out->light_raw = s_light_raw;
    out->valid = s_valid;
    out->period_ms = s_period_ms;
    out->last_update_ms = s_last_update_ms;
    out->error_count = s_error_count;
}

