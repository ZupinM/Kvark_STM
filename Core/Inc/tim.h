/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;

/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef hChargePumpTIM;

#undef  CHARGE_PUMP_PERIOD
#undef  CHARGE_PUMP_PULSE
#undef  MOTOR_PWM_PERIOD
#define CHARGE_PUMP_PERIOD  ((uint32_t)(SystemCoreClock / 5000) - 1) /*!< 5kHz  */
#define CHARGE_PUMP_PULSE  ((uint32_t)((SystemCoreClock / 5000) - 1) / 2 )
#define MOTOR_PWM_FREQUENCY 10000 //10kHz  //Old Kvark was 4kHz
#define MOTOR_PWM_PERIOD (SystemCoreClock / MOTOR_PWM_FREQUENCY)
#define MOTOR_PWM_MIN (MOTOR_PWM_PERIOD / 13)
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM6_Init(void);
void MX_TIM16_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
