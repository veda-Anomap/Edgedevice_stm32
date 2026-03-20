#include "app.h"
#include "mic.h"
#include "motor_ctrl.h"
#include "aht10.h"
#include "pcf8591.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern osMessageQueueId_t uart_rx_queueHandle;
extern osMessageQueueId_t control_queueHandle;
extern osMutexId_t uart_tx_mutexHandle;

typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL
} SystemMode_t;

static volatile SystemMode_t current_mode = MODE_AUTO;
static uint8_t rx_data = 0U;
static uint8_t rx_data_rpi = 0U;

#define PROTO_MAX_PAYLOAD 192U
#define PROTO_FRAME_QUEUE_LEN 8U
#define PROTO_RX_TIMEOUT_MS 100U
#define STATUS_REPLY_MIN_INTERVAL_MS 100U
#define MANUAL_CMD_MIN_INTERVAL_MS 90U
#define TDOA_FALLBACK_HOLD_MS 250U
#define UART1_RX_MIRROR_TO_UART2 1U
#define UART1_MIRROR_PAYLOAD_MAX 96U

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
static volatile uint32_t s_proto_last_byte_ms = 0U;

typedef struct {
    uint8_t cmd;
    uint32_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

static proto_frame_t s_frame_q[PROTO_FRAME_QUEUE_LEN];
static volatile uint8_t s_frame_q_head = 0U;
static volatile uint8_t s_frame_q_tail = 0U;
static volatile uint8_t s_frame_q_count = 0U;
static volatile uint8_t s_status_reply_pending = 0U;
static volatile uint32_t s_last_status_reply_ms = 0U;

/* UART1 (RPi) diagnostic counters */
static volatile uint32_t s_u1_isr_bytes = 0U;
static volatile uint32_t s_u1_frames = 0U;
static volatile uint32_t s_u1_queue_drop = 0U;
static volatile uint32_t s_u1_proto_oversize = 0U;
static volatile uint32_t s_u1_proto_invalid = 0U;
static volatile uint32_t s_u1_status_req = 0U;
static volatile uint32_t s_u1_motor_req = 0U;
static volatile uint32_t s_u1_ack_ok = 0U;
static volatile uint32_t s_u1_ack_fail = 0U;
static volatile uint32_t s_u1_err_total = 0U;
static volatile uint32_t s_u1_err_ore = 0U;
static volatile uint32_t s_u1_err_fe = 0U;
static volatile uint32_t s_u1_err_ne = 0U;
static volatile uint32_t s_u1_err_pe = 0U;
static volatile uint32_t s_u1_rearm_fail = 0U;
static volatile uint32_t s_u1_recover_ok = 0U;
static volatile uint32_t s_u1_recover_fail = 0U;
static volatile char s_auto_ctrl_src = 'D'; /* D: detect_dir, T: tdoa */

/* Coalesce manual move bursts: keep only the latest movement command */
static uint8_t s_pending_move_cmd = 0U;
static uint32_t s_last_move_apply_ms = 0U;

static void uart2_mirror_uart1_frame(uint8_t cmd, const uint8_t *payload, uint32_t len)
{
#if UART1_RX_MIRROR_TO_UART2
    char payload_txt[UART1_MIRROR_PAYLOAD_MAX + 1U];
    uint32_t copy_len = 0U;
    const char *tail = "";

    if ((payload != NULL) && (len > 0U)) {
        copy_len = (len > UART1_MIRROR_PAYLOAD_MAX) ? UART1_MIRROR_PAYLOAD_MAX : len;
        for (uint32_t i = 0U; i < copy_len; i++) {
            const uint8_t ch = payload[i];
            payload_txt[i] = ((ch >= 32U) && (ch <= 126U)) ? (char)ch : '.';
        }
        payload_txt[copy_len] = '\0';
        if (len > copy_len) {
            tail = "...";
        }
    } else {
        payload_txt[0] = '\0';
    }

    char line[220];
    int n = snprintf(line, sizeof(line),
                     "[U1->STM] CMD:0x%02X LEN:%lu PAYLOAD:%s%s\r\n",
                     (unsigned)cmd, (unsigned long)len, payload_txt, tail);
    if (n > 0) {
        const uint16_t tx_len = (n < (int)sizeof(line)) ? (uint16_t)n : (uint16_t)(sizeof(line) - 1U);
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, tx_len, 50U);
    }
#else
    (void)cmd;
    (void)payload;
    (void)len;
#endif
}

/* Convert PWM range to 0~180 degree scale */
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

    uint8_t locked = 0U;
    if ((uart_tx_mutexHandle != NULL) && (osKernelGetState() == osKernelRunning)) {
        if (osMutexAcquire(uart_tx_mutexHandle, osWaitForever) == osOK) {
            locked = 1U;
        }
    }

    (void)HAL_UART_Transmit(&huart1, header, sizeof(header), 100U);
    if (len > 0U) {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)json, len, 100U);
    }

    if (locked != 0U) {
        (void)osMutexRelease(uart_tx_mutexHandle);
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
    if (len > PROTO_MAX_PAYLOAD) return;

    __disable_irq();
    if (s_frame_q_count < PROTO_FRAME_QUEUE_LEN) {
        proto_frame_t *slot = &s_frame_q[s_frame_q_head];
        slot->cmd = cmd;
        slot->len = len;
        if ((len > 0U) && (payload != NULL)) {
            memcpy(slot->payload, payload, len);
        }
        s_frame_q_head = (uint8_t)((s_frame_q_head + 1U) % PROTO_FRAME_QUEUE_LEN);
        s_frame_q_count++;
    } else {
        /* Queue full: count dropped frame as invalid for diagnostics */
        s_u1_proto_invalid++;
    }
    __enable_irq();
}

static void proto_rx_reset(void)
{
    s_proto_state = PROTO_RX_WAIT_CMD;
    s_proto_cmd = 0U;
    s_proto_len = 0U;
    s_proto_idx = 0U;
    s_proto_last_byte_ms = 0U;
}

static void proto_rx_feed_byte(uint8_t b)
{
    switch (s_proto_state) {
    case PROTO_RX_WAIT_CMD:
        if ((b != 0x04U) && (b != 0x05U)) {
            s_u1_proto_invalid++;
            break;
        }
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
            s_u1_proto_oversize++;
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
    if (s_frame_q_count > 0U) {
        const proto_frame_t *slot = &s_frame_q[s_frame_q_tail];
        *cmd = slot->cmd;
        *len = slot->len;
        if (*len > 0U) {
            memcpy(payload, slot->payload, *len);
        }
        s_frame_q_tail = (uint8_t)((s_frame_q_tail + 1U) % PROTO_FRAME_QUEUE_LEN);
        s_frame_q_count--;
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

static void process_protocol_frame(uint8_t cmd, const uint8_t *payload, uint32_t len, uint32_t now_ms)
{
    if (cmd == 0x05U) {
        if (len == 0U) {
            s_u1_status_req++;
            if ((now_ms - s_last_status_reply_ms) >= STATUS_REPLY_MIN_INTERVAL_MS) {
                proto_send_status_packet();
                s_last_status_reply_ms = now_ms;
                s_status_reply_pending = 0U;
            } else {
                s_status_reply_pending = 1U;
            }
        } else {
            s_u1_proto_invalid++;
            proto_uart1_send_packet(0x05U, "");
        }
        return;
    }

    if (cmd == 0x04U) {
        s_u1_motor_req++;
        char cmd_text[16] = {0};
        const char motor_cmd = parse_motor_command(payload, len, cmd_text, sizeof(cmd_text));
        if (motor_cmd != 0) {
            const uint8_t queued = push_control_command((uint8_t)motor_cmd);
            if (queued != 0U) {
                s_u1_ack_ok++;
            } else {
                s_u1_ack_fail++;
            }
            proto_send_motor_ack(queued, cmd_text);
        } else {
            s_u1_proto_invalid++;
            s_u1_ack_fail++;
            proto_send_motor_ack(0U, "invalid");
        }
        return;
    }

    s_u1_proto_invalid++;
}

static void uart1_recover_rx_error(uint32_t err_flags)
{
    /* FE/NE/ORE/PE error recovery:
     * HAL clear macros internally perform the SR->DR clear sequence.
     * Do recovery only when needed to reduce impact on RX throughput. */
    if ((err_flags & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE |
                      HAL_UART_ERROR_NE  | HAL_UART_ERROR_PE)) != 0U) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    if (HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1) == HAL_OK) {
        s_u1_recover_ok++;
        return;
    }

    s_u1_rearm_fail++;

    /* If rearm fails in busy state, abort receive and rearm once again */
    (void)HAL_UART_AbortReceive_IT(&huart1);

    if ((err_flags & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE |
                      HAL_UART_ERROR_NE  | HAL_UART_ERROR_PE)) != 0U) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    if (HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1) == HAL_OK) {
        s_u1_recover_ok++;
    } else {
        s_u1_rearm_fail++;
        s_u1_recover_fail++;
    }
}

void app_on_uart1_byte(uint8_t b)
{
    const uint32_t now = HAL_GetTick();

    if ((s_proto_state != PROTO_RX_WAIT_CMD) &&
        ((now - s_proto_last_byte_ms) > PROTO_RX_TIMEOUT_MS)) {
        s_u1_proto_invalid++;
        proto_rx_reset();
    }
    s_proto_last_byte_ms = now;
    proto_rx_feed_byte(b);
}

static void drain_protocol_queue(uint32_t now_ms)
{
    uint8_t frame_cmd = 0U;
    uint32_t frame_len = 0U;
    uint8_t frame_payload[PROTO_MAX_PAYLOAD];

    while (proto_pop_frame(&frame_cmd, frame_payload, &frame_len) != 0U) {
        s_u1_frames++;
        uart2_mirror_uart1_frame(frame_cmd, frame_payload, frame_len);
        process_protocol_frame(frame_cmd, frame_payload, frame_len, now_ms);
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

static uint8_t is_move_command(uint8_t cmd)
{
    return (cmd == 'W' || cmd == 'w' ||
            cmd == 'A' || cmd == 'a' ||
            cmd == 'S' || cmd == 's' ||
            cmd == 'D' || cmd == 'd') ? 1U : 0U;
}

static uint8_t is_mode_command(uint8_t cmd)
{
    return (cmd == 'O' || cmd == 'o' || cmd == 'F' || cmd == 'f') ? 1U : 0U;
}

static void drain_control_queue(uint32_t now_ms)
{
    if (control_queueHandle == NULL) return;

    uint8_t cmd = 0U;
    while (osMessageQueueGet(control_queueHandle, &cmd, NULL, 0U) == osOK) {
        if (is_mode_command(cmd) != 0U) {
            /* Apply mode switch immediately */
            s_pending_move_cmd = 0U;
            apply_control_command(cmd);
            continue;
        }

        if (is_move_command(cmd) != 0U) {
            /* For move bursts, keep only the latest command (coalescing) */
            s_pending_move_cmd = cmd;
            continue;
        }

        /* Future extension commands are applied immediately */
        apply_control_command(cmd);
    }

    if ((s_pending_move_cmd != 0U) &&
        ((now_ms - s_last_move_apply_ms) >= MANUAL_CMD_MIN_INTERVAL_MS)) {
        apply_control_command(s_pending_move_cmd);
        s_last_move_apply_ms = now_ms;
        s_pending_move_cmd = 0U;
    }
}

void app_init(void)
{
    motor_ctrl_init(&htim3);
    mic_init();
    mic_tdoa_enable(1U);
    aht10_init(&hi2c1, 2000U);
    pcf8591_init(&hi2c1, 200U);

    /* Enable first UART RX interrupt (1 byte command) */
    (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    /* RPi protocol RX interrupt (frame by frame, 1 byte feed) */
    (void)HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1);

    printf("Starting DMA Audio System...\r\n");
}

#ifdef HAL_ADC_MODULE_ENABLED
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mic_on_dma_complete(hadc);
}
#endif

#ifdef HAL_I2S_MODULE_ENABLED
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    mic_on_i2s_rx_complete(hi2s);
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        (void)push_control_command(rx_data);
        (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);
        return;
    }

    if (huart == &huart1) {
        s_u1_isr_bytes++;
        if (uart_rx_queueHandle != NULL) {
            if (osMessageQueuePut(uart_rx_queueHandle, &rx_data_rpi, 0U, 0U) != osOK) {
                s_u1_queue_drop++;
            }
        }
        if (HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1) != HAL_OK) {
            s_u1_rearm_fail++;
        }
        return;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        const uint32_t e = huart->ErrorCode;
        s_u1_err_total++;
        if ((e & HAL_UART_ERROR_ORE) != 0U) s_u1_err_ore++;
        if ((e & HAL_UART_ERROR_FE) != 0U) s_u1_err_fe++;
        if ((e & HAL_UART_ERROR_NE) != 0U) s_u1_err_ne++;
        if ((e & HAL_UART_ERROR_PE) != 0U) s_u1_err_pe++;

        uart1_recover_rx_error(e);
        return;
    }

    if (huart == &huart2) {
        (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

void app_control_loop(void)
{
    const uint32_t nowm = HAL_GetTick();
    mic_tdoa_process(nowm);
    static uint32_t last_tdoa_ok_ms = 0U;

    /* Process completed UART1 frames in ControlTask context */
    drain_protocol_queue(nowm);

    if ((s_status_reply_pending != 0U) &&
        ((nowm - s_last_status_reply_ms) >= STATUS_REPLY_MIN_INTERVAL_MS)) {
        proto_send_status_packet();
        s_last_status_reply_ms = nowm;
        s_status_reply_pending = 0U;
    }

    /* Apply UART/protocol commands only in ControlTask */
    drain_control_queue(nowm);

    if (current_mode == MODE_AUTO) {
        mic_tdoa_debug_t tdbg = {0};
        mic_get_tdoa_debug(&tdbg);

        if (tdbg.valid != 0U) {
            last_tdoa_ok_ms = nowm;
            s_auto_ctrl_src = 'T';
            motor_ctrl_track_pan_tdoa(nowm, tdbg.alpha_deg_x10);
        } else if ((nowm - last_tdoa_ok_ms) <= TDOA_FALLBACK_HOLD_MS) {
            s_auto_ctrl_src = 'T';
            /* keep last tdoa-tracked position for a short hold window */
        } else if (mic_is_calibrated()) {
            s_auto_ctrl_src = 'D';
            const uint32_t lock_until_ms = motor_ctrl_get_lock_until_ms();
            const char detect_dir = mic_process(nowm, lock_until_ms);
            motor_ctrl_process(nowm, detect_dir);
        } else {
            s_auto_ctrl_src = '-';
        }
    } else {
        s_auto_ctrl_src = 'M';
        motor_ctrl_manual_process(nowm);
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
    if (nowm - last_dbg >= 500U) {
        uint32_t u1_q_used = 0U;
        if (uart_rx_queueHandle != NULL) {
            u1_q_used = osMessageQueueGetCount(uart_rx_queueHandle);
        }

        if (current_mode == MODE_AUTO) {
            mic_debug_t dbg = {0};
            mic_tdoa_debug_t tdbg = {0};
            aht10_data_t th;
            pcf8591_data_t light;
            char detect_dir = '-';

            if (mic_is_calibrated()) {
                mic_get_debug(&dbg);
                detect_dir = dbg.detect_dir;
            }
            mic_get_tdoa_debug(&tdbg);
            aht10_get_data(&th);
            pcf8591_get_data(&light);

            const int32_t temp_abs = (th.temperature_c_x100 < 0) ? -th.temperature_c_x100 : th.temperature_c_x100;
            const char temp_sign = (th.temperature_c_x100 < 0) ? '-' : '+';

            printf("I2S_L:%4lu I2S_R:%4lu | FINAL_L:%4lu FINAL_R:%4lu | DIR:%c | "
                   "PAN:%3lddeg TILT:%3lddeg | SRC:%c | T:%c%ld.%02ldC H:%lu.%02lu%% LIGHT:%3lu | "
                   "TDOA[V:%u L:%ld T:%ldus A:%ld.%01lddeg C:%u] | "
                   "U1[Q:%lu IN:%lu FR:%lu DR:%lu ER:%lu O:%lu F:%lu N:%lu P:%lu "
                   "IV:%lu OV:%lu R:%lu RO:%lu RF:%lu]\r\n",
                   (unsigned long)dbg.adc_avg_l, (unsigned long)dbg.adc_avg_r,
                    (unsigned long)dbg.sig_l, (unsigned long)dbg.sig_r,
                    detect_dir,
                    (long)pan_deg, (long)tilt_deg,
                     s_auto_ctrl_src,
                     temp_sign, (long)(temp_abs / 100), (long)(temp_abs % 100),
                      (unsigned long)(th.humidity_rh_x100 / 100U), (unsigned long)(th.humidity_rh_x100 % 100U),
                      (unsigned long)light.light_raw,
                     (unsigned)tdbg.valid,
                     (long)tdbg.lag_samples,
                     (long)tdbg.tau_us,
                     (long)(tdbg.alpha_deg_x10 / 10),
                     (long)((tdbg.alpha_deg_x10 < 0) ? -(tdbg.alpha_deg_x10 % 10) : (tdbg.alpha_deg_x10 % 10)),
                     (unsigned)tdbg.confidence_q8,
                     (unsigned long)u1_q_used,
                     (unsigned long)s_u1_isr_bytes,
                    (unsigned long)s_u1_frames,
                    (unsigned long)s_u1_queue_drop,
                    (unsigned long)s_u1_err_total,
                    (unsigned long)s_u1_err_ore,
                    (unsigned long)s_u1_err_fe,
                    (unsigned long)s_u1_err_ne,
                    (unsigned long)s_u1_err_pe,
                    (unsigned long)s_u1_proto_invalid,
                    (unsigned long)s_u1_proto_oversize,
                    (unsigned long)s_u1_rearm_fail,
                    (unsigned long)s_u1_recover_ok,
                    (unsigned long)s_u1_recover_fail);
        } else {
            pcf8591_data_t light;
            pcf8591_get_data(&light);
            printf("MODE:MANUAL | PAN:%3lddeg TILT:%3lddeg | PAN_PWM:%u TILT_PWM:%u | LIGHT:%3lu | "
                   "U1[Q:%lu IN:%lu FR:%lu DR:%lu ER:%lu O:%lu F:%lu N:%lu P:%lu "
                   "IV:%lu OV:%lu R:%lu RO:%lu RF:%lu]\r\n",
                   (long)pan_deg, (long)tilt_deg, pan_pwm, tilt_pwm, (unsigned long)light.light_raw,
                   (unsigned long)u1_q_used,
                   (unsigned long)s_u1_isr_bytes,
                   (unsigned long)s_u1_frames,
                   (unsigned long)s_u1_queue_drop,
                   (unsigned long)s_u1_err_total,
                   (unsigned long)s_u1_err_ore,
                   (unsigned long)s_u1_err_fe,
                   (unsigned long)s_u1_err_ne,
                   (unsigned long)s_u1_err_pe,
                   (unsigned long)s_u1_proto_invalid,
                   (unsigned long)s_u1_proto_oversize,
                   (unsigned long)s_u1_rearm_fail,
                   (unsigned long)s_u1_recover_ok,
                   (unsigned long)s_u1_recover_fail);
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
//
