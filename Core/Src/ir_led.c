#include "ir_led.h"

/* PB0, PB1, PB2 are configured as GPIO output in MX_GPIO_Init(). */
#define IR_LED_PORT     GPIOB
#define IR_LED_ALL_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2)

void IR_LED_All_On(void)
{
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_ALL_PINS, GPIO_PIN_SET);
}

void IR_LED_All_Off(void)
{
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_ALL_PINS, GPIO_PIN_RESET);
}
