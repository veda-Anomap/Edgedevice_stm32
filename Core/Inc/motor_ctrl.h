#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* PAN (left-right) PWM range */
#define PAN_LEFT    1210U
#define PAN_RIGHT   1810U
#define PAN_CENTER  1510U

/* TILT (up-down) PWM range */
#define TILT_UP     1210U
#define TILT_DOWN   1810U
#define TILT_CENTER 1510U

void motor_ctrl_init(TIM_HandleTypeDef *htim);
void motor_ctrl_process(uint32_t now_ms, char detect_dir);
void motor_ctrl_manual_process(uint32_t now_ms);
uint32_t motor_ctrl_get_lock_until_ms(void);

/* Manual mode control API */
void motor_ctrl_enter_manual(void);
void motor_ctrl_enter_auto(void);
void manual_move_pan(int step);
void manual_move_tilt(int step);

uint16_t motor_ctrl_get_pan_pwm(void);
uint16_t motor_ctrl_get_tilt_pwm(void);
