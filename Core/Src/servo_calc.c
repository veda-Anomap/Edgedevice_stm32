#include "main.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

// printf 리다이렉트가 이미 있으면 생략 가능
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

/**
 * 센터 탐색: 1450~1550을 step 간격으로 출력하고 2초 유지
 * 너는 그때 서보가 "가만히/덜덜/천천히 도는지" 보고 센터 후보를 고름.
 */
void ServoCal_Run(void)
{
    const uint16_t start = 1490;
    const uint16_t end   = 1520;
    const uint16_t step  = 10;      // 5us 단위(필요시 10us로)
    const uint32_t hold_ms = 2000; // 2초 유지

    printf("\r\n[ServoCal] start\r\n");
    printf("Watch servo. Find value where it stays still / most stable.\r\n");

    for (uint16_t us = start; us <= end; us += step)
    {
        servo_set_us(us);
        printf("TEST us=%u\r\n", us);
        HAL_Delay(hold_ms);
    }

    // 끝나면 1500으로 돌려놓기(원하면 제거 가능)
    servo_set_us(1500);
    printf("[ServoCal] done\r\n");
}
