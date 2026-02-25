#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 팬(좌우) PWM 범위 */
#define PAN_LEFT    1200U
#define PAN_RIGHT   1820U
#define PAN_CENTER  1520U

/* 틸트(상하) PWM 범위 */
#define TILT_UP     1200U
#define TILT_DOWN   1800U
#define TILT_CENTER 1500U

void motor_ctrl_init(TIM_HandleTypeDef *htim);
void motor_ctrl_process(uint32_t now_ms, char detect_dir);
uint32_t motor_ctrl_get_lock_until_ms(void);

/* 수동 모드 제어 API */
void motor_ctrl_enter_manual(void);
void motor_ctrl_enter_auto(void);
void manual_move_pan(int step);
void manual_move_tilt(int step);

uint16_t motor_ctrl_get_pan_pwm(void);
uint16_t motor_ctrl_get_tilt_pwm(void);
