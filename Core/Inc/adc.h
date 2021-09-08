/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include "../Inc/Shared_Libraries/bldc.h"

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
#if DEVICE == PICO
#define ADC_BUFFER_LENGTH  8
#else
#define ADC_BUFFER_LENGTH  16
#endif

//Voltage measuring points
#define HALL   0
#define SUPPLY  1
#define SUPPLY_UNFILTERED  2
#define BATTERY  3
#define CURRENT_A  4
#define CURRENT_A_UNFILTERED  5
#define CURRENT_B  6
#define CURRENT_B_UNFILTERED  7
#define VFOCUS 8
#define HFOCUS 9
#define TEMPERATURE 10

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

void StartADC_DMA_Sequence(void);
float GetAnalogValues(unsigned char measuring_point);
void SetADC_BatteryMeas(void);
void ClearADC_BatteryMeas(void);
uint8_t MotorSelectI(unsigned char motor);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
