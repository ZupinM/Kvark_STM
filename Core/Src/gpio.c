/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Xbee_RESET_Pin|LED_GR_Pin|LED_BL_Pin|DIVIDER_SENSOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_B_1L_Pin|MOTOR_A_3L_Pin|V_SELECT_A_Pin|MOTOR_A_2L_Pin
                          |MOTOR_B_2L_Pin|MOTOR_B_3L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, V_SELECT_B_Pin|MOTOR_A_1L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RD_GPIO_Port, LED_RD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_A_2H_Pin|MOTOR_B_2H_Pin|MOTOR_B_3H_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = Xbee_RESET_Pin|LED_GR_Pin|LED_BL_Pin|DIVIDER_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = MOTOR_B_1L_Pin|MOTOR_A_3L_Pin|V_SELECT_A_Pin|MOTOR_A_2L_Pin
                          |MOTOR_B_2L_Pin|MOTOR_B_3L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin */
  GPIO_InitStruct.Pin = V_SELECT_B_Pin|MOTOR_A_1L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = HALL_A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HALL_A1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = HALL_A2_Pin|HALL_A3_Pin|HALL_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin */
  GPIO_InitStruct.Pin = HALL_B3_Pin|HALL_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin */
  GPIO_InitStruct.Pin = END_SW_A_LO_Pin|END_SW_A_HI_Pin|END_SW_B_HI_Pin|BT1_Pin
                          |BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_RD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = END_SW_B_LO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(END_SW_B_LO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = MOTOR_A_2H_Pin|MOTOR_B_2H_Pin|MOTOR_B_3H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

uint32_t LEDTimeouts[4];
uint32_t LEDStates;
uint32_t LEDStates_old;

void set_LED(uint8_t color, uint8_t state, uint16_t timeout){

	if(state == LED_ON){
		if(timeout && !(LEDStates & 1<<color)){ //set timeout only when LED was OFF before
			LEDTimeouts[color] = timeout;
		}else{
			LEDTimeouts[color] = 0;
		}
		LEDStates |= 1<<color;
	}else if(state == LED_OFF){
		LEDStates &= ~(1<<color);
	}
	else if(state == COUNTDOWN){
		for(int i=0 ; i<4 ; i++){
			if(LEDTimeouts[i]){
				LEDTimeouts[i]--;
			}
			else if(LEDTimeouts[i] == 1){
				LEDStates &= ~(1<<color);
			}
		}
	}

	if((LEDStates & 1<<RED) != (LEDStates_old & 1<<RED)){
		HAL_GPIO_WritePin(LED_RD_GPIO_Port, LED_RD_Pin, !(LEDStates & 1<<RED) );
	}
	if((LEDStates & 1<<GREEN) != (LEDStates_old & 1<<GREEN)){
		HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, !(LEDStates & 1<<GREEN) );
	}
	if((LEDStates & 1<<BLUE) != (LEDStates_old & 1<<BLUE)){
		HAL_GPIO_WritePin(LED_BL_GPIO_Port, LED_BL_Pin, !(LEDStates & 1<<BLUE) );
	}

	if((LEDStates & 1<<WHITE) != (LEDStates_old & 1<<WHITE)){
		HAL_GPIO_WritePin(LED_RD_GPIO_Port, LED_RD_Pin, !(LEDStates & 1<<WHITE) );
		HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, !(LEDStates & 1<<WHITE) );
		HAL_GPIO_WritePin(LED_BL_GPIO_Port, LED_BL_Pin, !(LEDStates & 1<<WHITE) );
	}

}

uint8_t voltage_select      = 12;
uint8_t voltage_select_old  = 12;

void HallVoltage(){

  if (voltage_select != voltage_select_old){
    if(voltage_select == 5){
    	HAL_GPIO_WritePin(V_SELECT_A_GPIO_Port, V_SELECT_A_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(V_SELECT_B_GPIO_Port, V_SELECT_B_Pin, GPIO_PIN_SET);
    }
    else if(voltage_select == 12){
    	HAL_GPIO_WritePin(V_SELECT_A_GPIO_Port, V_SELECT_A_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(V_SELECT_B_GPIO_Port, V_SELECT_B_Pin, GPIO_PIN_RESET);
    }
    else if (voltage_select == 20){
    	HAL_GPIO_WritePin(V_SELECT_A_GPIO_Port, V_SELECT_A_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(V_SELECT_B_GPIO_Port, V_SELECT_B_Pin, GPIO_PIN_RESET);
    }

    voltage_select_old = voltage_select;
  }
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
