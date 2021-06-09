/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
  RTC_DateTypeDef sDateOnStartup = {0};

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  HAL_RTC_GetDate(&hrtc, &sDateOnStartup, RTC_FORMAT_BIN);
  if (sDateOnStartup.Year > 0){
	  return;							//RTC time already counting on battery
  }

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

time_t_ time; //Solar time
time_t_ GMTTime;


time_t_ tmpt;

char m_chMonthTable[] = {
  0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};


//void RTC_softwareCorrection(int secsPerDay){
//  float add;
//  int val;
//
//  add = (float)secsPerDay * 0.555549;
//  add = -add;
//  val= 48000 + (int)add;
//
//  if(val > 65535) val = 65535;
//  if(val<1000)    val = 1000;
//
//  TIM3->ARR = (uint16_t)val;
//}



char isYearNotLeap(uint32_t uiYear) {
  if(!(uiYear % 100)) {
    return(uiYear % 400);
  }
  return(uiYear % 4);
}


void TicksToDateTime(uint32_t ulTimeTicks, time_t_ *dt){
unsigned int uiDateYear;
  unsigned char ucDateMonth;
  unsigned long long ulTimeTicksTmp;
  unsigned long long ulTimeTicksMem;


  ucDateMonth = 1;
  uiDateYear = 1970;

  ulTimeTicksTmp = 0;
  ulTimeTicksMem = 0;

  /* find year */
  while(ulTimeTicksTmp < ulTimeTicks) {
    ulTimeTicksMem = ulTimeTicksTmp;

    ulTimeTicksTmp += (365*24*60*60);

    if(!isYearNotLeap(uiDateYear++)){
      ulTimeTicksTmp += (24*60*60);
    }
  }

  if(uiDateYear > 1970) {
    uiDateYear -= 1;
  }

  ulTimeTicks -= ulTimeTicksMem;
  ulTimeTicksTmp = 0;

  /* find month */
  while(ulTimeTicksTmp <= ulTimeTicks) {
    ulTimeTicksMem = ulTimeTicksTmp;

    ulTimeTicksTmp += (m_chMonthTable[ucDateMonth]*24*60*60);

    if(ucDateMonth == 2) {
      if(!isYearNotLeap(uiDateYear)) {
        ulTimeTicksTmp += (24*60*60);
      }
    }

    ucDateMonth += 1;
  }

  if(ucDateMonth > 1) {
    ucDateMonth -= 1;
  }

  /* find day */
  ulTimeTicks -= ulTimeTicksMem;

  dt->date = (unsigned char)((ulTimeTicks/24/60/60)+1);
  dt->month = ucDateMonth;
  dt->year = (unsigned short)uiDateYear;

  dt->hours = (unsigned char)((ulTimeTicks/3600)%24);
  dt->minutes = (unsigned char)((ulTimeTicks/60)%60);
  dt->seconds = (unsigned char)(ulTimeTicks%60);
}


uint32_t DateTimeToTicks(time_t_ *dt) {
  unsigned int uiDateYear;
  unsigned char ucDateMonth;

  unsigned long ulElapsedDays = 0;
  uint32_t Ticks;

  for(ucDateMonth = 1; ucDateMonth < dt->month; ucDateMonth++) {
    ulElapsedDays += m_chMonthTable[ucDateMonth];

    if(ucDateMonth == 2) {
      if(!isYearNotLeap(dt->year)) {
        ulElapsedDays += 1;
      }
    }
  }

  ulElapsedDays += (unsigned long)(dt->date-1);
  ulElapsedDays += ((dt->year-1970)*365);

  for(uiDateYear = 1970; uiDateYear < dt->year; uiDateYear++) {
    if(!isYearNotLeap(uiDateYear)) {
      ulElapsedDays += 1;
    }
  }

  Ticks =  (unsigned long)dt->seconds;
  Ticks += ((unsigned long)dt->minutes*60);
  Ticks += ((unsigned long)dt->hours*60*60);
  Ticks += (ulElapsedDays*24*60*60);
  return Ticks;
}


void getSolarDateTime(time_t_ *dt){
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  time_t_ tmpTime;
  tmpTime.seconds = sTime.Seconds;
  tmpTime.minutes = sTime.Minutes;
  tmpTime.hours   = sTime.Hours;

  tmpTime.date   = sDate.Date;
  tmpTime.month  = sDate.Month;
  tmpTime.year   = sDate.Year + 2000;

  TicksToDateTime(DateTimeToTicks(&tmpTime) + (longitude*240), dt);
}

void setSolarDateTime(time_t_ *dt){
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  TicksToDateTime(DateTimeToTicks(dt) - (longitude*240), dt);

  /*if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }*/
  sTime.Seconds = dt->seconds;
  sTime.Minutes = dt->minutes;
  sTime.Hours = dt->hours;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  sDate.Date = dt->date;
  sDate.Month = dt->month;
  sDate.Year = dt->year - 2000;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

}

void setGMTDateTime(time_t_ *dt){
	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};
	  //TicksToDateTime(DateTimeToTicks(dt), dt);

	  /*if (HAL_RTC_Init(&hrtc) != HAL_OK)
	  {
	    Error_Handler();
	  }*/
	  sTime.Seconds = dt->seconds;
	  sTime.Minutes = dt->minutes;
	  sTime.Hours = dt->hours;
	  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	  sDate.Date = dt->date;
	  sDate.Month = dt->month;
	  sDate.Year = dt->year - 2000;
	  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

}


void getGMTDateTime(time_t_ *dt){
	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  dt->seconds = sTime.Seconds;
	  dt->minutes = sTime.Minutes;
	  dt->hours   = sTime.Hours;

	  dt->date   = sDate.Date;
	  dt->month  = sDate.Month;
	  dt->year   = sDate.Year + 2000;
}

void setSolarSeconds(uint32_t sec){
  getSolarDateTime(&tmpt);
  if(sec>59) return;
  tmpt.seconds = sec;
  setSolarDateTime(&tmpt);
}

void setSolarMinutes(uint32_t min){
  getSolarDateTime(&tmpt);
  if(min>59) return;
  tmpt.minutes = min;
  setSolarDateTime(&tmpt);
}

void setSolarHours(uint32_t hours){
  getSolarDateTime(&tmpt);
  if(hours > 23)return;
  tmpt.hours = hours;
  setSolarDateTime(&tmpt);
}

void setSolarMday(uint32_t mday){
  getSolarDateTime(&tmpt);
  if(mday>31) return;
  tmpt.date = mday;
  setSolarDateTime(&tmpt);
}

void setSolarMonth(uint32_t mon){
  getSolarDateTime(&tmpt);
  if(mon>12) return;
  tmpt.month = mon;
  setSolarDateTime(&tmpt);
}

void setSolarYear(uint32_t year){
  getSolarDateTime(&tmpt);
  if(year<2000)return;
  tmpt.year = year;
  setSolarDateTime(&tmpt);
}

void setGMTSeconds(uint32_t sec){
  getGMTDateTime(&tmpt);
  if(sec>59) return;
  tmpt.seconds = sec;
  setGMTDateTime(&tmpt);
}

void setGMTMinutes(uint32_t min){
  getGMTDateTime(&tmpt);
  if(min>59) return;
  tmpt.minutes = min;
  setGMTDateTime(&tmpt);
}

void setGMTHours(uint32_t hours){
  getGMTDateTime(&tmpt);
  if(hours > 23)return;
  tmpt.hours = hours;
  setGMTDateTime(&tmpt);
}

void setGMTMday(uint32_t mday){
  getGMTDateTime(&tmpt);
  if(mday>31) return;
  tmpt.date = mday;
  setGMTDateTime(&tmpt);
}

void setGMTMonth(uint32_t mon){
  getGMTDateTime(&tmpt);
  if(mon>12) return;
  tmpt.month = mon;
  setGMTDateTime(&tmpt);
}

void setGMTYear(uint32_t year){
  getGMTDateTime(&tmpt);
  if(year<2000)return;
  tmpt.year = year;
  setGMTDateTime(&tmpt);
}

void Time_add(time_t_ *dt,  int sec ){
  uint32_t newseconds = DateTimeToTicks(dt);
  newseconds+=sec;
  TicksToDateTime(newseconds, dt);
}

void Time_round(time_t_ *dt, unsigned int sec ){
  uint32_t newseconds;
  newseconds = DateTimeToTicks(dt);
  newseconds -= newseconds%sec;
  TicksToDateTime(newseconds, dt);
}

int Time_higherEq(time_t_ *dt1, time_t_ *dt2){
  unsigned int time1=0;
  unsigned int time2=0;
  time1+=dt1->hours*60*60;
  time1+=dt1->minutes *60;
  time1+=dt1->seconds;

  time2+=dt2->hours*60*60;
  time2+=dt2->minutes *60;
  time2+=dt2->seconds;

 if(time1>=time2) return 1;
 return 0;
}

int Time_lower(time_t_ *dt1, time_t_ *dt2){
  unsigned int time1=0;
  unsigned int time2=0;
  time1+=dt1->hours*60*60;
  time1+=dt1->minutes *60;
  time1+=dt1->seconds;

  time2+=dt2->hours*60*60;
  time2+=dt2->minutes *60;
  time2+=dt2->seconds;

 if(time1<time2) return 1;
 return 0;
}





int rtc_find_correction(int secsPerDay){//return nearest prescaler difference
  int val = abs(secsPerDay);

  val = (int)((float)val * 0.3792548);

  if(val > 32000)  val = 32000;
  if(val<0)       val = 0;

  if(secsPerDay<0) return val;
  return -val;
}


void rtc_apply_correction(){
  uint8_t correction_min = 0;
  uint8_t correction_sec = 0;
  if(rtc_correction > 59){
    correction_min = rtc_correction / 60;
    correction_sec = (int)rtc_correction % 60;
  }else
  correction_sec = rtc_correction;

  getSolarDateTime(&tmpt);
  tmpt.seconds += correction_sec;
  tmpt.minutes += correction_min;
  setSolarDateTime(&tmpt);

  getGMTDateTime(&tmpt);
  tmpt.seconds += correction_sec;
  tmpt.minutes += correction_min;
  //setGMTDateTime(&tmpt);

}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
