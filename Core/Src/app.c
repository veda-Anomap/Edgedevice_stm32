#include "app.h"
#include "mic.h"
#include "motor_ctrl.h"
#include "aht10.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL
} SystemMode_t;

static volatile SystemMode_t current_mode = MODE_AUTO;
static uint8_t rx_data = 0U;
static uint8_t mode_cmd_prefix = 0U; /* 'a' or 'm' when waiting second char */

static void handle_uart_command(uint8_t cmd)
{
    /* 2-char mode commands:
     * - "at" -> auto mode
     * - "ma" -> manual mode
     */
    if (mode_cmd_prefix == 'a') {
        mode_cmd_prefix = 0U;
        if (cmd == 't' || cmd == 'T') {
            current_mode = MODE_AUTO;
            motor_ctrl_enter_auto();
            return;
        }
        /* fallthrough: current char is handled normally */
    } else if (mode_cmd_prefix == 'm') {
        mode_cmd_prefix = 0U;
        if (cmd == 'a' || cmd == 'A') {
            current_mode = MODE_MANUAL;
            motor_ctrl_enter_manual();
            return;
        }
        /* fallthrough: current char is handled normally */
    }

    if (cmd == 'a') {
        mode_cmd_prefix = 'a';
        return;
    }
    if (cmd == 'm') {
        mode_cmd_prefix = 'm';
        return;
    }

    if (current_mode == MODE_MANUAL) {
        /* Manual commands */
        if (cmd == 'W' || cmd == 'w') {
            manual_move_tilt(-50);
            return;
        }
        if (cmd == 'S' || cmd == 's') {
            manual_move_tilt(+50);
            return;
        }
        if (cmd == 'A') {
            manual_move_pan(-50);
            return;
        }
        if (cmd == 'D' || cmd == 'd') {
            manual_move_pan(+50);
            return;
        }
    }
}

void app_init(void)
{
    motor_ctrl_init(&htim3);
    mic_init(&hadc1);
    aht10_init(&hi2c1, 2000U);

    /* Enable first UART RX interrupt (1 byte command) */
    (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);

    printf("Starting DMA Audio System...\r\n");
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mic_on_dma_complete(hadc);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) return;

    handle_uart_command(rx_data);

    /* Re-arm 1 byte RX interrupt */
    (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);
}

void app_loop(void)
{
    const uint32_t nowm = HAL_GetTick();
    char detect_dir = '-';

    aht10_process(nowm);

    if (current_mode == MODE_AUTO) {
        if (mic_is_calibrated()) {
            const uint32_t lock_until_ms = motor_ctrl_get_lock_until_ms();
            detect_dir = mic_process(nowm, lock_until_ms);
            motor_ctrl_process(nowm, detect_dir);
        }
    }

    static uint32_t last_dbg = 0U;
    if (nowm - last_dbg >= 200U) {
        mic_debug_t dbg = {0};
        aht10_data_t th;
        const char dir_out = (current_mode == MODE_MANUAL) ? 'M' : detect_dir;

        if (mic_is_calibrated()) {
            mic_get_debug(&dbg);
        }
        aht10_get_data(&th);

        const int32_t temp_abs = (th.temperature_c_x100 < 0) ? -th.temperature_c_x100 : th.temperature_c_x100;
        const char temp_sign = (th.temperature_c_x100 < 0) ? '-' : '+';

        printf("ADC_L:%4lu ADC_R:%4lu | FINAL_L:%4lu FINAL_R:%4lu | DIR:%c | "
               "T:%c%ld.%02ldC H:%lu.%02lu%%\r\n",
               (unsigned long)dbg.adc_avg_l, (unsigned long)dbg.adc_avg_r,
               (unsigned long)dbg.sig_l, (unsigned long)dbg.sig_r,
               dir_out,
               temp_sign, (long)(temp_abs / 100), (long)(temp_abs % 100),
               (unsigned long)(th.humidity_rh_x100 / 100U), (unsigned long)(th.humidity_rh_x100 % 100U));
        last_dbg = nowm;
    }
}
