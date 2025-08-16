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
	__attribute__((unused)) void* argp = (void*) ((&st));

	return;
}

void set_log_level(u8 level) {
	LEVEL = level;
}

void ilog(u8 level, const char* str, ...) {
	u32 len = strlen(str);
	if(len > STRING_BOUND && !(level & LEVEL)) return;
	u32 ptr = 0;
	char buffer[len];
	memset(buffer, 0, len);

	void* argp = (void*) &str;
	argp += 4;

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

	u8 skip = 0;
	while(*str) {
		if(*str == '%')
		switch(*(str+1)) {
		case '%':
			buffer[ptr] = '%';
			skip = 1;
			break;
		case 'c':
			buffer[ptr] = *((char*) argp);
			argp++;
			skip = 1;
			break;
		case 's':
			memcpy(buffer, (char*) argp, skip = strlen((char*) argp));
			ptr += skip - 1;
			skip = 1;
			break;
		} else {
			buffer[ptr] = *str;
		}

		ptr++;
		str += 1 + skip;
		skip = 0;
	}
	//TODO: string bufer har safar syscall chaqirmaslik uchun (long latency)

	strncpy(prefix, temp, PREFIX_SIZE);
	printf("%s %s\r\n", prefix, buffer);
}
