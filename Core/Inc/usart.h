/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "Shared_Libraries/SX1278.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

#define RS485_ENABLED		0
#define TX_INTERRUPT		1		/* 0 if TX uses polling, 1 interrupt driven. */
#define MODEM_TEST		0

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04

#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

#define BUFSIZE_LONG	288


/* RS485 mode definition. */
#define RS485_NMMEN		(0x1<<0)
#define RS485_RXDIS		(0x1<<1)
#define RS485_AADEN		(0x1<<2)
#define RS485_SEL		(0x1<<3)
#define RS485_DCTRL		(0x1<<4)
#define RS485_OINV		(0x1<<5)

#define MODBUS_PACKET_RECIVED (1<<0)
#define MODBUS_DISCARD_PACKET	(1<<1)
#define MODBUS_ADDRESS_VALID  (1<<2)
#define MODBUS_RESTART				(1<<3)
#define MODBUS_CLEAR_MASK 		(~(MODBUS_PACKET_RECIVED|MODBUS_DISCARD_PACKET|MODBUS_ADDRESS_VALID))

#define UART_MODE_NONE 0
#define UART_MODE_RS485 (1 << 0)
#define UART_MODE_XBEE (1 << 1)

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void MX_USART2_UART_Init(void);
void UARTSend(uint8_t *BufferPtr, uint32_t Length);
void UART1Send(uint8_t *BufferPtr, uint32_t Length);

int modbus_discard(void);
int modbus_newRequest(void);
void modbus_ReqProcessed(void);

int modbus_discard1(void);
int modbus_newRequest1(void);
void modbus_ReqProcessed1(void);

int modbus_discard2(void);
int modbus_newRequest2(void);
void modbus_ReqProcessed2(void);

void reEnable_485_DMA_RX(void);
void UART_ChangeBaudRate(int baud);

void UART0ClearStatus(void);
void UART1ClearStatus(void);
void UART2ClearStatus(void);

void rs485_RTS_timeout(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
