/*
 * logger.h
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "type.h"

#define LEVEL_INFO		0x01
#define LEVEL_WARN		0x02
#define LEVEL_ERROR		0x04
#define LEVEL_SEVERE	0x08

#define PREFIX_SIZE 	15
#define STRING_BOUND	100

void ilog(u8, const char*, ...);
void set_log_level(u8);
void testfunc(char* st, ...);

#endif /* INC_LOGGER_H_ */
