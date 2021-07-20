/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

#define SPI_TIMEOUT 50

void LoRa_Interrupt(uint8_t enable_disable){
  if(enable_disable)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
  else
    NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/* USER CODE END 0 */

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PB4 (NJTRST)     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = LORA_SCK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(LORA_SCK_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LORA_MISO_Pin|LORA_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI3 DMA Init */
    /* SPI3_RX Init */
    hdma_spi3_rx.Instance = DMA2_Channel1;
    hdma_spi3_rx.Init.Request = DMA_REQUEST_3;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi3_rx);

    /* SPI3_TX Init */
    hdma_spi3_tx.Instance = DMA2_Channel2;
    hdma_spi3_tx.Init.Request = DMA_REQUEST_3;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi3_tx);

  /* USER CODE BEGIN SPI3_MspInit 1 */

    HAL_GPIO_WritePin(Xbee_RESET_GPIO_Port, Xbee_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PB4 (NJTRST)     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(LORA_SCK_GPIO_Port, LORA_SCK_Pin);

    HAL_GPIO_DeInit(GPIOB, LORA_MISO_Pin|LORA_MOSI_Pin);

    /* SPI3 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t TxBuffer[LoRa_MAX_PACKET];
uint8_t ReadTxBuffer[LoRa_MAX_PACKET];
void LoRa_SPIWrite(uint8_t addr, uint8_t *pcBuffer, uint8_t cNbBytes) {

	uint32_t timeout = 0;
	while (hspi3.State != HAL_SPI_STATE_READY && (timeout++ <= SPI_TIMEOUT * SystemCoreClock/1000) );
	TxBuffer[0] = (addr | 0x80);
	memcpy(TxBuffer + 1, pcBuffer, cNbBytes);
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)TxBuffer, cNbBytes + 1)  != HAL_OK)
    {
    	Error_Handler();
    }

}


void LoRa_SPIRead(uint8_t addr, uint8_t *pcBuffer, uint8_t cNbBytes) {

	uint32_t timeout = 0;
	while (hspi3.State != HAL_SPI_STATE_READY && (timeout++ <= SPI_TIMEOUT * SystemCoreClock/1000) );
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &addr, 1, SPI_TIMEOUT);
	if(HAL_SPI_TransmitReceive_DMA(&hspi3, pcBuffer, pcBuffer, cNbBytes) !=  HAL_OK)
    {
    	Error_Handler();
    }
	if(lora_int_stat == PACKET_RECEIVED_SPI_TRANSFER){
		return;	// dont wait for transaction complete (interrupt handling)
	}
	while (hspi3.State != HAL_SPI_STATE_READY && (timeout++ <= SPI_TIMEOUT * SystemCoreClock/1000) );
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
	if(transmission >= 1){	//Lora fifo loaded -> transmitt over RF
		SPI_TxFifo_cmplt = 1;
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
	if(lora_int_stat == PACKET_RECEIVED_SPI_TRANSFER){ //Lora fifo received -> reenable reception
		SPI_RxFifo_cmplt = 1;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	Error_Handler();
}


void LoRa_reset(void) {
  HAL_GPIO_WritePin(Xbee_RESET_GPIO_Port, Xbee_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(Xbee_RESET_GPIO_Port, Xbee_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
}

void spi_rx_fifo_clear(void) {
  //uint8_t dump;
 // while (LPC_SPI0->STAT & (1 << 0))
    //dump = LPC_SPI0->RXDAT;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
