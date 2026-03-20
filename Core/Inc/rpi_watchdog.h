#ifndef RPI_WATCHDOG_H
#define RPI_WATCHDOG_H

#include "main.h"
#include <stdint.h>

typedef struct {
    uint32_t grace_sec;
    uint32_t heartbeat_timeout_ms;
    uint32_t relay_cut_sec;
    uint32_t poll_ms;
    uint8_t uart_log_enable;
} rpi_watchdog_cfg_t;

void rpi_watchdog_init(UART_HandleTypeDef *dbg_uart, const rpi_watchdog_cfg_t *cfg);
void rpi_watchdog_log_boot_status(const IWDG_HandleTypeDef *hiwdg, uint32_t sys_miss_max);
void rpi_watchdog_task(void *argument);
uint32_t rpi_watchdog_get_alive_counter(void);

#endif /* RPI_WATCHDOG_H */

