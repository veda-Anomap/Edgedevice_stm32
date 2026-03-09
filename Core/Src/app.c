#include "app.h"
#include "mic.h"
#include "motor_ctrl.h"
#include "aht10.h"
#include "pcf8591.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern osMessageQueueId_t uart_rx_queueHandle;
extern osMessageQueueId_t control_queueHandle;

typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL
} SystemMode_t;

static volatile SystemMode_t current_mode = MODE_AUTO;
static uint8_t rx_data = 0U;
static uint8_t rx_data_rpi = 0U;

#define PROTO_MAX_PAYLOAD 192U

typedef enum {
    PROTO_RX_WAIT_CMD = 0,
    PROTO_RX_WAIT_LEN_0,
    PROTO_RX_WAIT_LEN_1,
    PROTO_RX_WAIT_LEN_2,
    PROTO_RX_WAIT_LEN_3,
    PROTO_RX_WAIT_PAYLOAD
} proto_rx_state_t;

static volatile proto_rx_state_t s_proto_state = PROTO_RX_WAIT_CMD;
static volatile uint8_t s_proto_cmd = 0U;
static volatile uint32_t s_proto_len = 0U;
static volatile uint32_t s_proto_idx = 0U;
static uint8_t s_proto_rx_buf[PROTO_MAX_PAYLOAD];

static volatile uint8_t s_frame_ready = 0U;
static uint8_t s_frame_cmd = 0U;
static uint32_t s_frame_len = 0U;
static uint8_t s_frame_payload[PROTO_MAX_PAYLOAD];

/* PWM 범위를 0~180도 각도로 선형 변환 */
static int32_t pwm_to_deg(uint16_t pwm, uint16_t min_pwm, uint16_t max_pwm)
{
    if (max_pwm <= min_pwm) return 0;
    if (pwm < min_pwm) pwm = min_pwm;
    if (pwm > max_pwm) pwm = max_pwm;
    return ((int32_t)(pwm - min_pwm) * 180) / (int32_t)(max_pwm - min_pwm);
}

static void proto_uart1_send_packet(uint8_t cmd, const char *json)
{
    if (json == NULL) return;

    const uint32_t len = (uint32_t)strlen(json);
    uint8_t header[5];
    header[0] = cmd;
    header[1] = (uint8_t)((len >> 24) & 0xFFU);
    header[2] = (uint8_t)((len >> 16) & 0xFFU);
    header[3] = (uint8_t)((len >> 8) & 0xFFU);
    header[4] = (uint8_t)(len & 0xFFU);

    (void)HAL_UART_Transmit(&huart1, header, sizeof(header), 100);
    if (len > 0U) {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)json, len, 100);
    }
}

static void proto_send_status_packet(void)
{
    aht10_data_t th = {0};
    pcf8591_data_t light = {0};
    mic_debug_t dbg = {0};
    char dir = '-';

    if (mic_is_calibrated()) {
        mic_get_debug(&dbg);
        dir = dbg.detect_dir;
    }
    aht10_get_data(&th);
    pcf8591_get_data(&light);

    const uint16_t tilt_pwm = motor_ctrl_get_tilt_pwm();
    const int32_t tilt_deg = pwm_to_deg(tilt_pwm, TILT_UP, TILT_DOWN);
    const int32_t t_abs = (th.temperature_c_x100 < 0) ? -th.temperature_c_x100 : th.temperature_c_x100;

    char temp_buf[16];
    if (th.temperature_c_x100 < 0) {
        (void)snprintf(temp_buf, sizeof(temp_buf), "-%ld.%02ld",
                       (long)(t_abs / 100), (long)(t_abs % 100));
    } else {
        (void)snprintf(temp_buf, sizeof(temp_buf), "%ld.%02ld",
                       (long)(t_abs / 100), (long)(t_abs % 100));
    }

    char json[160];
    (void)snprintf(json, sizeof(json),
                   "{\"tmp\":%s,\"hum\":%lu.%02lu,\"dir\":\"%c\",\"tilt\":%ld,\"light\":%lu}",
                   temp_buf,
                   (unsigned long)(th.humidity_rh_x100 / 100U),
                   (unsigned long)(th.humidity_rh_x100 % 100U),
                   dir,
                   (long)tilt_deg,
                   (unsigned long)light.light_raw);

    proto_uart1_send_packet(0x05U, json);
}

static void proto_send_motor_ack(uint8_t ok, const char *cmd_text)
{
    const char *mode_str = (current_mode == MODE_AUTO) ? "auto" : "manual";
    char json[96];
    (void)snprintf(json, sizeof(json),
                   "{\"ok\":%u,\"mode\":\"%s\",\"cmd\":\"%s\"}",
                   (unsigned)ok, mode_str, (cmd_text != NULL) ? cmd_text : "");
    proto_uart1_send_packet(0x04U, json);
}

static void proto_publish_frame(uint8_t cmd, const uint8_t *payload, uint32_t len)
{
    if (s_frame_ready != 0U) return;
    if (len > PROTO_MAX_PAYLOAD) return;

    s_frame_cmd = cmd;
    s_frame_len = len;
    if (len > 0U && payload != NULL) {
        memcpy(s_frame_payload, payload, len);
    }
    s_frame_ready = 1U;
}

static void proto_rx_reset(void)
{
    s_proto_state = PROTO_RX_WAIT_CMD;
    s_proto_cmd = 0U;
    s_proto_len = 0U;
    s_proto_idx = 0U;
}

static void proto_rx_feed_byte(uint8_t b)
{
    switch (s_proto_state) {
    case PROTO_RX_WAIT_CMD:
        s_proto_cmd = b;
        s_proto_len = 0U;
        s_proto_idx = 0U;
        s_proto_state = PROTO_RX_WAIT_LEN_0;
        break;

    case PROTO_RX_WAIT_LEN_0:
        s_proto_len = ((uint32_t)b << 24);
        s_proto_state = PROTO_RX_WAIT_LEN_1;
        break;

    case PROTO_RX_WAIT_LEN_1:
        s_proto_len |= ((uint32_t)b << 16);
        s_proto_state = PROTO_RX_WAIT_LEN_2;
        break;

    case PROTO_RX_WAIT_LEN_2:
        s_proto_len |= ((uint32_t)b << 8);
        s_proto_state = PROTO_RX_WAIT_LEN_3;
        break;

    case PROTO_RX_WAIT_LEN_3:
        s_proto_len |= (uint32_t)b;
        if (s_proto_len > PROTO_MAX_PAYLOAD) {
            proto_rx_reset();
            break;
        }
        if (s_proto_len == 0U) {
            proto_publish_frame(s_proto_cmd, NULL, 0U);
            proto_rx_reset();
        } else {
            s_proto_idx = 0U;
            s_proto_state = PROTO_RX_WAIT_PAYLOAD;
        }
        break;

    case PROTO_RX_WAIT_PAYLOAD:
        if (s_proto_idx < PROTO_MAX_PAYLOAD) {
            s_proto_rx_buf[s_proto_idx++] = b;
        }
        if (s_proto_idx >= s_proto_len) {
            proto_publish_frame(s_proto_cmd, s_proto_rx_buf, s_proto_len);
            proto_rx_reset();
        }
        break;

    default:
        proto_rx_reset();
        break;
    }
}

static uint8_t proto_pop_frame(uint8_t *cmd, uint8_t *payload, uint32_t *len)
{
    uint8_t has_frame = 0U;

    __disable_irq();
    if (s_frame_ready != 0U) {
        *cmd = s_frame_cmd;
        *len = s_frame_len;
        if (*len > 0U) {
            memcpy(payload, s_frame_payload, *len);
        }
        s_frame_ready = 0U;
        has_frame = 1U;
    }
    __enable_irq();

    return has_frame;
}

static char parse_motor_command(const uint8_t *payload, uint32_t len, char *cmd_text, uint32_t cmd_text_size)
{
    if (payload == NULL || len == 0U || cmd_text == NULL || cmd_text_size == 0U) return 0;
    if (len > PROTO_MAX_PAYLOAD) return 0;

    char msg[PROTO_MAX_PAYLOAD + 1U];
    memcpy(msg, payload, len);
    msg[len] = '\0';

    for (uint32_t i = 0U; i < len; i++) {
        msg[i] = (char)tolower((unsigned char)msg[i]);
    }

    const char *k = strstr(msg, "motor");
    if (k == NULL) return 0;

    const char *colon = strchr(k, ':');
    if (colon == NULL) return 0;
    colon++;
    while (*colon == ' ' || *colon == '\t') {
        colon++;
    }

    char token[16] = {0};
    uint32_t ti = 0U;
    if (*colon == '\"') {
        colon++;
        while (*colon != '\0' && *colon != '\"' && ti < (sizeof(token) - 1U)) {
            token[ti++] = *colon++;
        }
    } else {
        while (*colon != '\0' && *colon != ',' && *colon != '}' && ti < (sizeof(token) - 1U)) {
            token[ti++] = *colon++;
        }
    }
    token[ti] = '\0';

    (void)snprintf(cmd_text, cmd_text_size, "%s", token);

    /* RPi protocol allow-list: a/s/w/d/auto/unauto */
    if (strcmp(token, "auto") == 0) return 'o';
    if (strcmp(token, "unauto") == 0) return 'f';
    if (strcmp(token, "w") == 0) return 'w';
    if (strcmp(token, "a") == 0) return 'a';
    if (strcmp(token, "s") == 0) return 's';
    if (strcmp(token, "d") == 0) return 'd';

    return 0;
}

static uint8_t push_control_command(uint8_t cmd)
{
    if (control_queueHandle == NULL) return 0U;
    return (osMessageQueuePut(control_queueHandle, &cmd, 0U, 0U) == osOK) ? 1U : 0U;
}

static void process_protocol_frame(uint8_t cmd, const uint8_t *payload, uint32_t len)
{
    if (cmd == 0x05U) {
        if (len == 0U) {
            proto_send_status_packet();
        } else {
            proto_uart1_send_packet(0x05U, "");
        }
        return;
    }

    if (cmd == 0x04U) {
        char cmd_text[16] = {0};
        const char motor_cmd = parse_motor_command(payload, len, cmd_text, sizeof(cmd_text));
        if (motor_cmd != 0) {
            const uint8_t queued = push_control_command((uint8_t)motor_cmd);
            proto_send_motor_ack(queued, cmd_text);
        } else {
            proto_send_motor_ack(0U, "invalid");
        }
        return;
    }
}

void app_on_uart1_byte(uint8_t b)
{
    uint8_t frame_cmd = 0U;
    uint32_t frame_len = 0U;
    uint8_t frame_payload[PROTO_MAX_PAYLOAD];

    proto_rx_feed_byte(b);

    while (proto_pop_frame(&frame_cmd, frame_payload, &frame_len) != 0U) {
        process_protocol_frame(frame_cmd, frame_payload, frame_len);
    }
}

static void apply_control_command(uint8_t cmd)
{
    /* Mode command:
     * - 'o' (on): auto mode
     * - 'f' (off): manual mode
     */
    if (cmd == 'o' || cmd == 'O') {
        current_mode = MODE_AUTO;
        motor_ctrl_enter_auto();
        return;
    }
    if (cmd == 'f' || cmd == 'F') {
        current_mode = MODE_MANUAL;
        motor_ctrl_enter_manual();
        return;
    }

    /* Manual movement commands:
     * if a movement command arrives in AUTO mode,
     * switch to MANUAL first and then apply movement.
     */
    if (cmd == 'W' || cmd == 'w' || cmd == 'S' || cmd == 's' ||
        cmd == 'A' || cmd == 'a' || cmd == 'D' || cmd == 'd') {
        if (current_mode != MODE_MANUAL) {
            current_mode = MODE_MANUAL;
            motor_ctrl_enter_manual();
        }

        if (cmd == 'W' || cmd == 'w') {
            manual_move_tilt(-50);
            return;
        }
        if (cmd == 'S' || cmd == 's') {
            manual_move_tilt(+50);
            return;
        }
        if (cmd == 'A' || cmd == 'a') {
            manual_move_pan(-50);
            return;
        }
        if (cmd == 'D' || cmd == 'd') {
            manual_move_pan(+50);
            return;
        }
    }
}

static void drain_control_queue(void)
{
    if (control_queueHandle == NULL) return;

    uint8_t cmd = 0U;
    while (osMessageQueueGet(control_queueHandle, &cmd, NULL, 0U) == osOK) {
        apply_control_command(cmd);
    }
}

void app_init(void)
{
    motor_ctrl_init(&htim3);
    mic_init(&hadc1);
    aht10_init(&hi2c1, 2000U);
    pcf8591_init(&hi2c1, 200U);

    /* Enable first UART RX interrupt (1 byte command) */
    (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    /* RPi protocol RX interrupt (frame by frame, 1 byte feed) */
    (void)HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1);

    printf("Starting DMA Audio System...\r\n");
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mic_on_dma_complete(hadc);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        (void)push_control_command(rx_data);
        (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);
        return;
    }

    if (huart == &huart1) {
        if (uart_rx_queueHandle != NULL) {
            (void)osMessageQueuePut(uart_rx_queueHandle, &rx_data_rpi, 0U, 0U);
        }
        (void)HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1);
        return;
    }
}

void app_control_loop(void)
{
    const uint32_t nowm = HAL_GetTick();

    /* UART/프로토콜 명령은 ControlTask에서만 반영 */
    drain_control_queue();

    if (current_mode == MODE_AUTO) {
        if (mic_is_calibrated()) {
            const uint32_t lock_until_ms = motor_ctrl_get_lock_until_ms();
            const char detect_dir = mic_process(nowm, lock_until_ms);
            motor_ctrl_process(nowm, detect_dir);
        }
    }
}

void app_sensor_loop(void)
{
    const uint32_t nowm = HAL_GetTick();
    const uint16_t pan_pwm = motor_ctrl_get_pan_pwm();
    const uint16_t tilt_pwm = motor_ctrl_get_tilt_pwm();
    const int32_t pan_deg = pwm_to_deg(pan_pwm, PAN_LEFT, PAN_RIGHT);
    const int32_t tilt_deg = pwm_to_deg(tilt_pwm, TILT_UP, TILT_DOWN);

    aht10_process(nowm);
    pcf8591_process(nowm);

    static uint32_t last_dbg = 0U;
    if (nowm - last_dbg >= 200U) {
        if (current_mode == MODE_AUTO) {
            mic_debug_t dbg = {0};
            aht10_data_t th;
            pcf8591_data_t light;
            char detect_dir = '-';

            if (mic_is_calibrated()) {
                mic_get_debug(&dbg);
                detect_dir = dbg.detect_dir;
            }
            aht10_get_data(&th);
            pcf8591_get_data(&light);

            const int32_t temp_abs = (th.temperature_c_x100 < 0) ? -th.temperature_c_x100 : th.temperature_c_x100;
            const char temp_sign = (th.temperature_c_x100 < 0) ? '-' : '+';

            printf("ADC_L:%4lu ADC_R:%4lu | FINAL_L:%4lu FINAL_R:%4lu | DIR:%c | "
                   "PAN:%3lddeg TILT:%3lddeg | T:%c%ld.%02ldC H:%lu.%02lu%% LIGHT:%3lu\r\n",
                   (unsigned long)dbg.adc_avg_l, (unsigned long)dbg.adc_avg_r,
                   (unsigned long)dbg.sig_l, (unsigned long)dbg.sig_r,
                   detect_dir,
                   (long)pan_deg, (long)tilt_deg,
                   temp_sign, (long)(temp_abs / 100), (long)(temp_abs % 100),
                   (unsigned long)(th.humidity_rh_x100 / 100U), (unsigned long)(th.humidity_rh_x100 % 100U),
                   (unsigned long)light.light_raw);
        } else {
            pcf8591_data_t light;
            pcf8591_get_data(&light);
            printf("MODE:MANUAL | PAN:%3lddeg TILT:%3lddeg | PAN_PWM:%u TILT_PWM:%u | LIGHT:%3lu\r\n",
                   (long)pan_deg, (long)tilt_deg, pan_pwm, tilt_pwm, (unsigned long)light.light_raw);
        }
        last_dbg = nowm;
    }
}

void app_loop(void)
{
    /* Backward-compatible wrapper (non-RTOS or legacy call path) */
    app_sensor_loop();
    app_control_loop();
}
