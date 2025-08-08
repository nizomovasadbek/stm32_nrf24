/*
 * logger.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */


#include "stdio.h"
#include "logger.h"
#include "string.h"

uint8_t CONFIG = 0x00;

void logg(uint8_t level, const char* str) {
	__attribute__((unused)) char s[10];
	memset(s, 0, 10);

	printf("\r\n");
}

void log_init(uint8_t level) {
	CONFIG = level;
}
