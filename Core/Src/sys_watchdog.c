#include "sys_watchdog.h"
#include "rpi_watchdog.h"
#include "cmsis_os2.h"

#define SYS_MONITOR_PERIOD_MS_DEFAULT 1000U
#define SYS_MONITOR_MISS_MAX_DEFAULT     4U

static IWDG_HandleTypeDef *s_hiwdg = NULL;
static sys_watchdog_cfg_t s_cfg = {
    SYS_MONITOR_PERIOD_MS_DEFAULT,
    SYS_MONITOR_MISS_MAX_DEFAULT
};

void sys_watchdog_init(IWDG_HandleTypeDef *hiwdg, const sys_watchdog_cfg_t *cfg)
{
    s_hiwdg = hiwdg;
    if (cfg != NULL) {
        s_cfg = *cfg;
    }
}

void sys_watchdog_task(void *argument)
{
    (void)argument;

    uint32_t last_seen_counter = rpi_watchdog_get_alive_counter();
    uint32_t miss_count = 0U;

    for (;;) {
        osDelay(s_cfg.period_ms);

        const uint32_t alive_now = rpi_watchdog_get_alive_counter();

        if (alive_now != last_seen_counter) {
            last_seen_counter = alive_now;
            miss_count = 0U;

#ifdef HAL_IWDG_MODULE_ENABLED
            if (s_hiwdg != NULL) {
                (void)HAL_IWDG_Refresh(s_hiwdg);
            }
#endif
        } else {
            if (miss_count < 0xFFFFFFFFU) {
                miss_count++;
            }

            if (miss_count < s_cfg.miss_max) {
#ifdef HAL_IWDG_MODULE_ENABLED
                if (s_hiwdg != NULL) {
                    (void)HAL_IWDG_Refresh(s_hiwdg);
                }
#endif
            } else {
                /* Intentionally stop IWDG refresh to force hardware reset */
            }
        }
    }
}

