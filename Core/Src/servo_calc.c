#include "main.h"
#include <stdio.h>
#include "cmsis_os2.h"

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

// Skip if printf redirection is already defined elsewhere
//int _write(int file, char *ptr, int len)
//{
//    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
//    return len;
//}

static void servo_set_us(uint16_t us)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}

/* Use osDelay when RTOS is running, HAL_Delay before scheduler start */
static void delay_ms(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        (void)osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

/**
 * Center sweep: output 1450~1550 with step interval and hold 2 seconds each.
 * Observe where servo is most stable and choose that as center trim value.
 */
void ServoCal_Run(void)
{
    const uint16_t start = 1490;
    const uint16_t end   = 1520;
    const uint16_t step  = 10;      // test step (us)
    const uint32_t hold_ms = 2000;  // hold time per step

    printf("\r\n[ServoCal] start\r\n");
    printf("Watch servo. Find value where it stays still / most stable.\r\n");

    for (uint16_t us = start; us <= end; us += step)
    {
        servo_set_us(us);
        printf("TEST us=%u\r\n", us);
        delay_ms(hold_ms);
    }

    // Return to 1500 at end (remove if not needed)
    servo_set_us(1500);
    printf("[ServoCal] done\r\n");
}
