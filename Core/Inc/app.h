#pragma once
#include "stm32f4xx_hal.h"

void app_init(void);
void app_loop(void);
void app_on_uart1_byte(uint8_t b);
