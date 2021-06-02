/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM1_PRESCALER 5
#define BT3_Pin GPIO_PIN_2
#define BT3_GPIO_Port GPIOE
#define CHARGE_PUMP_Pin GPIO_PIN_6
#define CHARGE_PUMP_GPIO_Port GPIOE
#define Xbee_RESET_Pin GPIO_PIN_13
#define Xbee_RESET_GPIO_Port GPIOC
#define H_FOCUS_Pin GPIO_PIN_0
#define H_FOCUS_GPIO_Port GPIOC
#define V_FOCUS_Pin GPIO_PIN_1
#define V_FOCUS_GPIO_Port GPIOC
#define UVccHALL_Pin GPIO_PIN_2
#define UVccHALL_GPIO_Port GPIOC
#define AD_input_1_Pin GPIO_PIN_3
#define AD_input_1_GPIO_Port GPIOC
#define SNOW_Pin GPIO_PIN_0
#define SNOW_GPIO_Port GPIOA
#define MOTOR_I_SENS_B_Pin GPIO_PIN_1
#define MOTOR_I_SENS_B_GPIO_Port GPIOA
#define AD_input_2_Pin GPIO_PIN_3
#define AD_input_2_GPIO_Port GPIOA
#define USUPPLY_Pin GPIO_PIN_5
#define USUPPLY_GPIO_Port GPIOA
#define WIND_Pin GPIO_PIN_6
#define WIND_GPIO_Port GPIOA
#define MOTOR_I_SENS_A_Pin GPIO_PIN_7
#define MOTOR_I_SENS_A_GPIO_Port GPIOA
#define MOTOR_B_1L_Pin GPIO_PIN_0
#define MOTOR_B_1L_GPIO_Port GPIOB
#define MOTOR_A_3L_Pin GPIO_PIN_1
#define MOTOR_A_3L_GPIO_Port GPIOB
#define V_SELECT_A_Pin GPIO_PIN_2
#define V_SELECT_A_GPIO_Port GPIOB
#define V_SELECT_B_Pin GPIO_PIN_7
#define V_SELECT_B_GPIO_Port GPIOE
#define MOTOR_A_1L_Pin GPIO_PIN_8
#define MOTOR_A_1L_GPIO_Port GPIOE
#define MOTOR_A_1H_Pin GPIO_PIN_9
#define MOTOR_A_1H_GPIO_Port GPIOE
#define MOTOR_B_1H_Pin GPIO_PIN_11
#define MOTOR_B_1H_GPIO_Port GPIOE
#define MOTOR_A_3H_Pin GPIO_PIN_13
#define MOTOR_A_3H_GPIO_Port GPIOE
#define HALL_A1_Pin GPIO_PIN_15
#define HALL_A1_GPIO_Port GPIOE
#define HALL_A2_Pin GPIO_PIN_10
#define HALL_A2_GPIO_Port GPIOB
#define HALL_A3_Pin GPIO_PIN_11
#define HALL_A3_GPIO_Port GPIOB
#define HALL_B1_Pin GPIO_PIN_12
#define HALL_B1_GPIO_Port GPIOB
#define MOTOR_A_2L_Pin GPIO_PIN_13
#define MOTOR_A_2L_GPIO_Port GPIOB
#define MOTOR_B_2L_Pin GPIO_PIN_14
#define MOTOR_B_2L_GPIO_Port GPIOB
#define MOTOR_B_3L_Pin GPIO_PIN_15
#define MOTOR_B_3L_GPIO_Port GPIOB
#define HALL_B3_Pin GPIO_PIN_8
#define HALL_B3_GPIO_Port GPIOD
#define HALL_B2_Pin GPIO_PIN_9
#define HALL_B2_GPIO_Port GPIOD
#define END_SW_A_LO_Pin GPIO_PIN_10
#define END_SW_A_LO_GPIO_Port GPIOD
#define END_SW_A_HI_Pin GPIO_PIN_13
#define END_SW_A_HI_GPIO_Port GPIOD
#define END_SW_B_HI_Pin GPIO_PIN_14
#define END_SW_B_HI_GPIO_Port GPIOD
#define LED_RD_Pin GPIO_PIN_15
#define LED_RD_GPIO_Port GPIOD
#define LED_GR_Pin GPIO_PIN_6
#define LED_GR_GPIO_Port GPIOC
#define LED_BL_Pin GPIO_PIN_7
#define LED_BL_GPIO_Port GPIOC
#define END_SW_B_LO_Pin GPIO_PIN_8
#define END_SW_B_LO_GPIO_Port GPIOC
#define DIVIDER_SENSOR_Pin GPIO_PIN_9
#define DIVIDER_SENSOR_GPIO_Port GPIOC
#define MOTOR_A_2H_Pin GPIO_PIN_8
#define MOTOR_A_2H_GPIO_Port GPIOA
#define MOTOR_B_2H_Pin GPIO_PIN_9
#define MOTOR_B_2H_GPIO_Port GPIOA
#define MOTOR_B_3H_Pin GPIO_PIN_10
#define MOTOR_B_3H_GPIO_Port GPIOA
#define LORA_SCK_Pin GPIO_PIN_10
#define LORA_SCK_GPIO_Port GPIOC
#define BT1_Pin GPIO_PIN_0
#define BT1_GPIO_Port GPIOD
#define BT2_Pin GPIO_PIN_1
#define BT2_GPIO_Port GPIOD
#define USART2_TX_out2_Pin GPIO_PIN_5
#define USART2_TX_out2_GPIO_Port GPIOD
#define USART2_RX_out1_Pin GPIO_PIN_6
#define USART2_RX_out1_GPIO_Port GPIOD
#define RS485_RTS_Pin GPIO_PIN_3
#define RS485_RTS_GPIO_Port GPIOB
#define LORA_MISO_Pin GPIO_PIN_4
#define LORA_MISO_GPIO_Port GPIOB
#define LORA_MOSI_Pin GPIO_PIN_5
#define LORA_MOSI_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_6
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_7
#define RS485_RX_GPIO_Port GPIOB
#define BOOT_Pin GPIO_PIN_3
#define BOOT_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
