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

void set_log_level(u8 level) {
	LEVEL = level;
}

void ilog(u8 level, const char* str) {
	if(strlen(str) > STRING_BOUND && !(level & LEVEL)) return;
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

	strncpy(prefix, temp, PREFIX_SIZE);
	printf("%s %s\n", prefix, str);
}
