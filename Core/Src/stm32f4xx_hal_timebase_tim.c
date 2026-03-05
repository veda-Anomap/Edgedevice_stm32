/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL time base based on TIM11.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

TIM_HandleTypeDef htim11;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef clkconfig;
  uint32_t uwTimclock;
  uint32_t uwAPB2Prescaler = 0U;
  uint32_t uwPrescalerValue = 0U;
  uint32_t pFLatency;

  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  uwAPB2Prescaler = clkconfig.APB2CLKDivider;

  if (uwAPB2Prescaler == RCC_HCLK_DIV1) {
    uwTimclock = HAL_RCC_GetPCLK2Freq();
  } else {
    uwTimclock = 2U * HAL_RCC_GetPCLK2Freq();
  }

  /* TIM11 counter clock = 1 MHz */
  uwPrescalerValue = (uwTimclock / 1000000U) - 1U;

  htim11.Instance = TIM11;
  htim11.Init.Period = (1000000U / 1000U) - 1U; /* 1 ms tick */
  htim11.Init.Prescaler = uwPrescalerValue;
  htim11.Init.ClockDivision = 0U;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_TIM_Base_Start_IT(&htim11) != HAL_OK) {
    return HAL_ERROR;
  }

  if (TickPriority < (1UL << __NVIC_PRIO_BITS)) {
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  } else {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
}

