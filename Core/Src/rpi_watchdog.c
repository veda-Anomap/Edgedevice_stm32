#include "rpi_watchdog.h"
#include "cmsis_os2.h"
#include <stdarg.h>
#include <stdio.h>

#define RPI_WDG_GRACE_SEC_DEFAULT           60U
#define RPI_HEARTBEAT_TIMEOUT_MS_DEFAULT 10000U
#define RPI_RELAY_CUT_SEC_DEFAULT            3U
#define RPI_WDG_POLL_MS_DEFAULT             10U
#define RPI_WDG_UART_LOG_ENABLE_DEFAULT      1U

static volatile uint32_t s_alive_counter = 0U;
static volatile uint32_t s_last_heartbeat_tick = 0U;
static volatile uint32_t s_relay_reset_count = 0U;

static UART_HandleTypeDef *s_dbg_uart = NULL;
static rpi_watchdog_cfg_t s_cfg = {
    RPI_WDG_GRACE_SEC_DEFAULT,
    RPI_HEARTBEAT_TIMEOUT_MS_DEFAULT,
    RPI_RELAY_CUT_SEC_DEFAULT,
    RPI_WDG_POLL_MS_DEFAULT,
    RPI_WDG_UART_LOG_ENABLE_DEFAULT
};

static void rpi_wdg_log(const char *fmt, ...)
{
    if ((s_cfg.uart_log_enable == 0U) || (s_dbg_uart == NULL) || (fmt == NULL)) {
        return;
    }

    char msg[160];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    const uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(s_dbg_uart, (uint8_t *)msg, tx_len, 100U);
}

static void wait_seconds_and_mark_alive(uint32_t seconds)
{
    for (uint32_t i = 0U; i < seconds; i++) {
        s_alive_counter++;
        osDelay(1000U);
    }
}

static GPIO_PinState sync_heartbeat_baseline(void)
{
    const GPIO_PinState level = HAL_GPIO_ReadPin(RPI_HB_GPIO_Port, RPI_HB_Pin);
    s_last_heartbeat_tick = osKernelGetTickCount();
    return level;
}

void rpi_watchdog_init(UART_HandleTypeDef *dbg_uart, const rpi_watchdog_cfg_t *cfg)
{
    s_dbg_uart = dbg_uart;

    if (cfg != NULL) {
        s_cfg = *cfg;
    }
}

void rpi_watchdog_log_boot_status(const IWDG_HandleTypeDef *hiwdg, uint32_t sys_miss_max)
{
    const uint32_t bor  = (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)  != RESET) ? 1U : 0U;
    const uint32_t pin  = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  != RESET) ? 1U : 0U;
    const uint32_t por  = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)  != RESET) ? 1U : 0U;
    const uint32_t sw   = (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  != RESET) ? 1U : 0U;
    const uint32_t iwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) ? 1U : 0U;
    const uint32_t wwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) ? 1U : 0U;
    const uint32_t lpwr = (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) ? 1U : 0U;

    rpi_wdg_log("[RST] BOR:%lu PIN:%lu POR:%lu SW:%lu IWDG:%lu WWDG:%lu LPWR:%lu\r\n",
                (unsigned long)bor, (unsigned long)pin, (unsigned long)por,
                (unsigned long)sw, (unsigned long)iwdg, (unsigned long)wwdg,
                (unsigned long)lpwr);

    __HAL_RCC_CLEAR_RESET_FLAGS();

    if (hiwdg != NULL) {
        rpi_wdg_log("[IWDG] presc=%lu reload=%lu miss_max=%lu\r\n",
                    (unsigned long)hiwdg->Init.Prescaler,
                    (unsigned long)hiwdg->Init.Reload,
                    (unsigned long)sys_miss_max);
    }
}

uint32_t rpi_watchdog_get_alive_counter(void)
{
    return s_alive_counter;
}

void rpi_watchdog_task(void *argument)
{
    (void)argument;

    /* Active-high relay module: keep LOW in normal operation */
    HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_RESET);

    /* Initial grace period for RPi boot */
    wait_seconds_and_mark_alive(s_cfg.grace_sec);

    GPIO_PinState last_level = sync_heartbeat_baseline();
    uint32_t hb_edges = 0U;
    uint8_t hb_armed = 0U; /* Start timeout check only after first heartbeat edge is seen */

    rpi_wdg_log("[RPI-WDG] start: grace=%lus timeout=%lums cut=%lus\r\n",
                (unsigned long)s_cfg.grace_sec,
                (unsigned long)s_cfg.heartbeat_timeout_ms,
                (unsigned long)s_cfg.relay_cut_sec);
    rpi_wdg_log("[RPI-HB] waiting first edge (watchdog timeout disabled)\r\n");

    for (;;) {
        const uint32_t now = osKernelGetTickCount();
        const GPIO_PinState cur_level = HAL_GPIO_ReadPin(RPI_HB_GPIO_Port, RPI_HB_Pin);

        s_alive_counter++;

        if (cur_level != last_level) {
            last_level = cur_level;
            s_last_heartbeat_tick = now;
            hb_edges++;
            if (hb_armed == 0U) {
                hb_armed = 1U;
                rpi_wdg_log("[RPI-HB] first edge detected -> watchdog armed\r\n");
            }
            rpi_wdg_log("[RPI-HB] level=%lu edge=%lu link=OK\r\n",
                        (unsigned long)((cur_level == GPIO_PIN_SET) ? 1U : 0U),
                        (unsigned long)hb_edges);
        }

        if ((hb_armed != 0U) &&
            ((now - s_last_heartbeat_tick) > s_cfg.heartbeat_timeout_ms)) {
            rpi_wdg_log("[RPI-HB] timeout=%lums -> link=LOST, relay cut\r\n",
                        (unsigned long)s_cfg.heartbeat_timeout_ms);

            HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_SET);
            wait_seconds_and_mark_alive(s_cfg.relay_cut_sec);
            HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_RESET);

            s_relay_reset_count++;
            rpi_wdg_log("[RPI-WDG] relay restore done, reset_count=%lu, grace wait\r\n",
                        (unsigned long)s_relay_reset_count);

            wait_seconds_and_mark_alive(s_cfg.grace_sec);

            last_level = sync_heartbeat_baseline();
            hb_armed = 0U;
            rpi_wdg_log("[RPI-WDG] grace done, heartbeat monitoring restart\r\n");
            rpi_wdg_log("[RPI-HB] waiting first edge (watchdog timeout disabled)\r\n");
        }

        osDelay(s_cfg.poll_ms);
    }
}

