#ifndef SYS_WATCHDOG_H
#define SYS_WATCHDOG_H

#include "main.h"
#include <stdint.h>

typedef struct {
    uint32_t period_ms;
    uint32_t miss_max;
} sys_watchdog_cfg_t;

void sys_watchdog_init(IWDG_HandleTypeDef *hiwdg, const sys_watchdog_cfg_t *cfg);
void sys_watchdog_task(void *argument);

#endif /* SYS_WATCHDOG_H */

