/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];
extern bldc_motor bldc_motors[BLDC_MOTOR_COUNT];
extern bldc_misc  bldc_cfg;
uint32_t ADC_calibration_value;

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_64;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  if(init_main_state)
  {
	  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
	  {
		Error_Handler();
	  }
	  ADC_calibration_value = LL_ADC_GetCalibrationFactor(hadc1.Instance, ADC_SINGLE_ENDED);
  }
  else
  {
	  LL_ADC_SetCalibrationFactor(hadc1.Instance, ADC_SINGLE_ENDED, ADC_calibration_value);
  }
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN1
    PC1     ------> ADC1_IN2
    PC2     ------> ADC1_IN3
    PC3     ------> ADC1_IN4
    PA0     ------> ADC1_IN5
    PA1     ------> ADC1_IN6
    PA3     ------> ADC1_IN8
    PA5     ------> ADC1_IN10
    PA6     ------> ADC1_IN11
    PA7     ------> ADC1_IN12
    */
    GPIO_InitStruct.Pin = H_FOCUS_Pin|V_FOCUS_Pin|UVccHALL_Pin|AD_input_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SNOW_Pin|MOTOR_I_SENS_B_Pin|AD_input_2_Pin|USUPPLY_Pin
                          |WIND_Pin|MOTOR_I_SENS_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN1
    PC1     ------> ADC1_IN2
    PC2     ------> ADC1_IN3
    PC3     ------> ADC1_IN4
    PA0     ------> ADC1_IN5
    PA1     ------> ADC1_IN6
    PA3     ------> ADC1_IN8
    PA5     ------> ADC1_IN10
    PA6     ------> ADC1_IN11
    PA7     ------> ADC1_IN12
    */
    HAL_GPIO_DeInit(GPIOC, H_FOCUS_Pin|V_FOCUS_Pin|UVccHALL_Pin|AD_input_1_Pin);

    HAL_GPIO_DeInit(GPIOA, SNOW_Pin|MOTOR_I_SENS_B_Pin|AD_input_2_Pin|USUPPLY_Pin
                          |WIND_Pin|MOTOR_I_SENS_A_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint32_t BatteryMeas_cnt = 100000;
uint32_t ADCBatteryVoltage;

void StartADC_DMA_Sequence(void){
	if(HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	//HAL_DMA_Abort(&hdma_adc1);  // Reset DMA destination address
	  if(BatteryMeas_cnt++ > 100000){ //Measure battery voltage ~once per 20s
		  BatteryMeas_cnt = 0;
		  SetADC_BatteryMeas();
		  if(HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, 1) != HAL_OK)
		  {
			  Error_Handler();
		  }
		  return;
	  }else if(BatteryMeas_cnt == 1){
		  ADCBatteryVoltage = g_ADCBuffer[0];
		  ClearADC_BatteryMeas();
	  }
	if(HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, ADC_BUFFER_LENGTH) != HAL_OK)
	{
		Error_Handler();
	}

}

void ClearADC_BatteryMeas(void){

	if(HAL_ADC_DeInit(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	MX_ADC1_Init();

}
void SetADC_BatteryMeas(void){

	  if(HAL_ADC_DeInit(&hadc1) != HAL_OK){
		  Error_Handler();
  	  }
	  hadc1.Init.OversamplingMode = DISABLE;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_VBAT;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  LL_ADC_SetCalibrationFactor(hadc1.Instance, ADC_SINGLE_ENDED, ADC_calibration_value);

}

uint32_t Uavg;
uint32_t TempAvg;
uint32_t Iavg_A;
uint32_t Iavg_B;
uint32_t HFocavg;
uint32_t VFocavg;
uint32_t HALLavg;
uint32_t zeroCurrent_voltage_0;
uint32_t zeroCurrent_voltage_1;


float GetAnalogValues(unsigned char measuring_point) {

  if(Uavg == 0){	//First measurment unfiltered
    Uavg = g_ADCBuffer[0];
  }
  Uavg = Uavg + ( (int)(g_ADCBuffer[1] - Uavg)*0.01);//integrator
  Iavg_A = Iavg_A + ( (int)((g_ADCBuffer[2] - zeroCurrent_voltage_0) - Iavg_A)*0.01);//integrator
  TempAvg = TempAvg + ( (int)(g_ADCBuffer[3] - TempAvg)*0.01);//integrator
#if DEVICE == KVARK
  Iavg_B = Iavg_B + ( (int)((g_ADCBuffer[4] - zeroCurrent_voltage_1) - Iavg_B)*0.01);//integrator
#endif
#if DEVICE != PICO  //No Hall voltage measurment on PICO
  HALLavg = HALLavg + ( (int)(g_ADCBuffer[5] - HALLavg)*0.01);//integrator
  VFocavg = VFocavg + ( (int)(g_ADCBuffer[6] - VFocavg)*0.01);//integrator
  HFocavg = HFocavg + ( (int)(g_ADCBuffer[7] - HFocavg)*0.01);//integrator
#endif

  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING)){ // get adc current reading in inactive states, to compensate op.amp offset
    zeroCurrent_voltage_0 += (int)(g_ADCBuffer[2] - zeroCurrent_voltage_0)*0.001;
    Iavg_A = 0;
  }
#if DEVICE == KVARK
  if(!(bldc_motors[1].status & BLDC_STATUS_MOVING)){
    zeroCurrent_voltage_1 += (int)(g_ADCBuffer[4] - zeroCurrent_voltage_1)*0.001;
    Iavg_B = 0;
  }
#endif


  switch(measuring_point){
    case SUPPLY :
      return (float)Uavg  / (bldc_cfg.UConvertRatio * 64.2);
    case SUPPLY_UNFILTERED :
      return (float)g_ADCBuffer[1] / (bldc_cfg.UConvertRatio * 64.2);
    case BATTERY :
    	return (float)ADCBatteryVoltage / 330;
    case TEMPERATURE:
      return __HAL_ADC_CALC_TEMPERATURE(TempAvg, 3300, ADC_RESOLUTION12b);
    case CURRENT_A:
    	return (float)Iavg_A / (bldc_cfg.IConvertRatio * 152);
    case CURRENT_A_UNFILTERED:
    	return (float)g_ADCBuffer[2] / (bldc_cfg.IConvertRatio * 152);
#if DEVICE == KVARK
    case CURRENT_B:
    	return (float)Iavg_B / (bldc_cfg.IConvertRatio * 152);
    case CURRENT_B_UNFILTERED:
    	return (float)g_ADCBuffer[4] / (bldc_cfg.IConvertRatio * 152);
#endif
#if DEVICE != PICO
    case HALL :
      return (float)HALLavg  / (bldc_cfg.HConvertRatio * 64.6);
    case VFOCUS :
    	return (float)VFocavg  /  256;
    case HFOCUS :
    	return (float)HFocavg  /  256;
#endif
    default: return 0;
  }
}

uint8_t MotorSelectI(unsigned char motor){
	if(motor == 0)
		return CURRENT_A;
	else
		return CURRENT_B;
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
