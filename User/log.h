/**
  ******************************************************************************
  * @file    log.h
  * @author  Application Team
	* @Version V1.0.0
  * @Date    1-April-2019
  * @brief   Header file of log module.
  ******************************************************************************
  */



#ifndef __CS32L010_LOG_H
#define __CS32L010_LOG_H

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "cs32l010_hal.h"

/* Choose one from below */
#define	LOG_METHOD_SERIAL
//#define	LOG_METHOD_RAM

#ifdef LOG_METHOD_SERIAL
	/* Serial port baud rate */
	#define LOG_SERIAL_BPS 9600
#endif

#ifdef LOG_METHOD_RAM
	/* Choose a valid address without any confliction */
	#define	LOG_RAM_ADDRESS		0x20001000
	#define LOG_RAM_CHAR		((volatile char *)(LOG_RAM_ADDRESS))
#endif


void UartInit(void);
void logout(bool success);
void panic(const char* func);
#endif /* __XM32F103_LOG_H */
