/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#define SystemFrequency SystemCoreClock
volatile uint8_t UARTTxEmpty0 = 1;
volatile uint8_t UARTTxEmpty1 = 1;
volatile uint8_t UARTTxEmpty2 = 1;
volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t UARTBuffer1[BUFSIZE];
volatile uint8_t UARTBuffer2[BUFSIZE];
volatile int rxTimeout0;
volatile int rxTimeout1;
volatile int rxTimeout2;
volatile uint32_t UARTCount0 = 0;
volatile uint32_t UARTtxCount0;
volatile uint32_t UARTtxCount1;
volatile uint32_t UARTtxCount2;
         uint8_t *BufferTXPtr0;
volatile uint8_t *BufferTXPtr1;
volatile uint8_t *BufferTXPtr2;
volatile uint32_t UARTCount1 = 0;
volatile uint32_t UARTCount2 = 0;
volatile uint32_t UARTCount3 = 0;
volatile uint8_t ModbusState0;
volatile uint8_t ModbusState1;
volatile uint8_t ModbusState2;
uint16_t flow_ctrl_hangup_timer = 0;
unsigned char uartMode;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)UARTBuffer0, BUFSIZE) != HAL_OK) //Start receiving
  {
    Error_Handler();
  }
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);	//Disable Parity Error interrupt
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_FE);	//Disable Framing Error interrupt
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_ORE);	//Disable Overrun Error interrupt
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //Enable UART Idle interrupt
  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB3 (JTDO/TRACESWO)     ------> USART1_DE
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|RS485_TX_Pin|RS485_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD3     ------> USART2_CTS
    PD4     ------> USART2_RTS
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|USART2_TX_out2_Pin|USART2_RX_out1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX
    PD12     ------> USART3_RTS
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB3 (JTDO/TRACESWO)     ------> USART1_DE
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|RS485_TX_Pin|RS485_RX_Pin);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD3     ------> USART2_CTS
    PD4     ------> USART2_RTS
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3|GPIO_PIN_4|USART2_TX_out2_Pin|USART2_RX_out1_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX
    PD12     ------> USART3_RTS
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  //UartReady = SET;


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	volatile uint32_t tmp;                  	// volatile to prevent optimizations
	tmp = huart1.Instance->ISR;                  // Read status register and data reg to clear RX flag
	tmp = huart1.Instance->RDR;
	(void) tmp;									// only to not have the compiler warning (variable not used)

	UARTCount0 = BUFSIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    ModbusState0 |= MODBUS_PACKET_RECIVED;

    if(HAL_UART_DMAStop(&huart1) ) //Stop receiving
    {
      Error_Handler();
    }
    //Reception is re-enabled in main

}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}



void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
	  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)BufferPtr, Length*2)!= HAL_OK) //Set length to double and treat HalfCplt interrupt as Transfer Complete
	  {
	    Error_Handler();
	  }
}

void UART1Send(uint8_t *BufferPtr, uint32_t Length)
{

}

int modbus_newRequest()
{
  if (uartMode == UART_MODE_RS485)
    return (ModbusState0 & MODBUS_PACKET_RECIVED) ? 1:0;
  else if (uartMode == UART_MODE_XBEE)
    return (ModbusState1 & MODBUS_PACKET_RECIVED) ? 1:0;
  else
	return 0;
}
int modbus_newRequest1()
{
  return (ModbusState1 & MODBUS_PACKET_RECIVED) ? 1:0;
}

int modbus_newRequest2()
{
  return (ModbusState2 & MODBUS_PACKET_RECIVED) ? 1:0;
}


int modbus_discard()
{
  if (uartMode == UART_MODE_RS485)
    return (ModbusState0 & MODBUS_DISCARD_PACKET) ? 1:0;
  else if (uartMode == UART_MODE_XBEE)
    return (ModbusState1 & MODBUS_DISCARD_PACKET) ? 1:0;
  else
	return 0;
}

int modbus_discard1()
{
  return (ModbusState1 & MODBUS_DISCARD_PACKET) ? 1:0;
}


int modbus_discard2()
{
  return (ModbusState2 & MODBUS_DISCARD_PACKET) ? 1:0;
}

void modbus_ReqProcessed()
{
  if (uartMode == UART_MODE_RS485) {
    ModbusState0 &= MODBUS_CLEAR_MASK;
    UARTCount0 = 0;
  }
  else if (uartMode == UART_MODE_XBEE) {
    ModbusState1 &= MODBUS_CLEAR_MASK;

    UARTCount1 = 0;
  }
  //uartMode = UART_MODE_NONE;
}

void reEnable_485_DMA_RX(void){

	if (huart1.RxState == HAL_UART_STATE_READY && hdma_usart1_tx.State != HAL_DMA_STATE_BUSY){	//Reenable reception, when DMA is stoped in HAL_UART_RxCpltCallback
	    if(HAL_UART_DMAStop(&huart1) ) //stop again to prevent errors
	    {
	      Error_Handler();
	    }
 	    if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)UARTBuffer0, BUFSIZE) != HAL_OK) 	//Start receiving
 	    {
 	      Error_Handler();
 	    }
    }

}

void UART_ChangeBaudRate(int baud){
	HAL_UART_DMAStop(&huart1);
    HAL_UART_DeInit(&huart1);
    huart1.Init.BaudRate = baud;
    if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
      Error_Handler();
    }
	if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)UARTBuffer0, BUFSIZE) != HAL_OK) 	//Start receiving
	{
	  Error_Handler();
	}
}

void modbus_ReqProcessed1()
{
  ModbusState1 &= MODBUS_CLEAR_MASK;
  UARTCount1 = 0;
}

void modbus_ReqProcessed2()
{
  ModbusState2 &= MODBUS_CLEAR_MASK;
 // UARTCount2 = 0;
}


void UART0ClearStatus()
{
 // LPC_USART0->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}

void UART1ClearStatus()
{
 // LPC_USART1->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}


void UART2ClearStatus()
{
 // LPC_USART2->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
