/*
 * logger.h
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "stdint.h"

#define LEVEL_INFO		0x00
#define LEVEL_WARN		0x01
#define LEVEL_ERROR		0x02
#define LEVEL_SEVERE	0x04

void log_init(uint8_t);
void logg(uint8_t, const char*);

#endif /* INC_LOGGER_H_ */
