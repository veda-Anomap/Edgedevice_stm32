#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

void motor_ctrl_init(TIM_HandleTypeDef *htim);
void motor_ctrl_process(uint32_t now_ms, char detect_dir);
uint32_t motor_ctrl_get_lock_until_ms(void);
