/*
 * logger.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */


#include "stdio.h"
#include "logger.h"
#include "string.h"

u8 LEVEL = 0;

void testfunc(char* st, ...) {
	void* argp = (void*) ((&st)+4);

	return;
}

void set_log_level(u8 level) {
	LEVEL = level;
}

void ilog(u8 level, const char* str, ...) {
	if(strlen(str) > STRING_BOUND && !(level & LEVEL)) return;

	void* argp = (void*) &level;
	argp += 2;

	char prefix[PREFIX_SIZE];
	memset(prefix, 0, PREFIX_SIZE);
	char* temp;

	switch(level) {
	case LEVEL_INFO:
		temp = "[INFO] ";
		break;
	case LEVEL_WARN:
		temp = "[WARNING] ";
		break;
	case LEVEL_ERROR:
		temp = "[ERROR] ";
		break;
	case LEVEL_SEVERE:
		temp = "[SEVERE] ";
		break;
	default:
		temp = "[UNKNOWN] ";
		break;
	}

	while(*str) {
		if(*str == '%') {
			// TODO: printf logikasi
		}
	}
	//TODO: string bufer har safar syscall chaqirmaslik uchun (long latency)

	strncpy(prefix, temp, PREFIX_SIZE);
	printf("%s %s\r\n", prefix, str);
}
