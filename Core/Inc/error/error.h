/*
 * error.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */

#ifndef INC_ERROR_ERROR_H_
#define INC_ERROR_ERROR_H_


#include "main.h"
#include "cmsis_os.h"
#include "type.h"

#define ERROR_LED_PORT	USER_LED_GPIO_Port
#define ERROR_LED_PIN	USER_LED_Pin

#define PRINT

#define READYOK				0x00
#define ERROR_QUEUEFULL		0x01
#define ERROR_TRANSMIT		0x02
#define ERROR_CHECKSUM		0x04
#define ERROR_HARDFAULT		0x08

#define SHORT_DELAY			100
#define LONG_DELAY			500

void error_poll(  void  );
void error_add(  u8 error_type  );
void _pulse(  u32,  u32  );

#endif
