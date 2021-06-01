/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */

extern uint32_t cflags;

#define RTC_USE_INTERNAL 1
#define time_t_O_TICS(_hour,_min) ((_hour*60*60) + (_min*60))

typedef struct {
  uint32_t seconds;
  uint32_t minutes;
  uint32_t hours;
  uint32_t day;
  uint32_t date;
  uint32_t month;
  uint32_t year;
} time_t_;
#include "Shared_Libraries/suntracer.h"
char isYearNotLeap(uint32_t uiYear);
void getDateTime(time_t_ *dt);
void getSolarDateTime(time_t_ *dt);
void getGMTDateTime(time_t_ *dt);
uint32_t  getSolar_Ticks();
uint32_t  getGMT_Ticks();

uint32_t  DateTimeToTicks(time_t_ *dt) ;
void TicksToDateTime(uint32_t ulTimeTicks, time_t_ *dt);

void setSolarDateTime(time_t_ *dt);

void setSolarTimeTics(uint32_t ticks);
void setSolarSeconds(uint32_t sec);
void setSolarMinutes(uint32_t min);
void setSolarHours(uint32_t hours);
void setSolarMday(uint32_t mday);
void setSolarMonth(uint32_t mon);
void setSolarYear(uint32_t year);


void setGMTSeconds(uint32_t sec);
void setGMTMinutes(uint32_t min);
void setGMTHours(uint32_t hours);
void setGMTMday(uint32_t mday);
void setGMTMonth(uint32_t mon);
void setGMTYear(uint32_t year);


void Time_add(time_t_ *dt,  int sec );
void Time_round(time_t_ *dt, unsigned int sec );
int Time_higherEq(time_t_ *dt1, time_t_ *dt2);
int Time_lower(time_t_ *dt1, time_t_ *dt2);

uint32_t TimeToTics(time_t_ *dt1);
uint32_t TimeStrToTime(char *str, time_t_ * t);
uint32_t TimeStrToTics(char *str);
uint32_t getdoyFromTics(uint32_t tics);
uint32_t getdoy(time_t_ *dt);

void rtc_applay_correction(int secsPerDay);
int rtc_find_correction(int secsPerDay);
void RTC_softwareCorrection(int secsPerDay);
void rtc_apply_correction();

void rtc_init(void);
void RTC_SetCounter(uint32_t ticks);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
