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

void testfunc( char* st, ... ) {
	__attribute__((unused)) void* argp = ( void* ) (( &st ));

	return;
}

void set_log_level( u8 level ) {
	LEVEL = level;
}


//under construction
void ilog( u8 level, const char* str, ... ) {
	u32 len = strlen(  str  );
	if( len > STRING_BOUND && !( level & LEVEL ) ) return;

	void* argp = ( void* ) &str;
	argp += 4;

	char prefix[PREFIX_SIZE];
	memset( prefix, 0, PREFIX_SIZE );
	char* temp;

	switch( level ) {
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

	printf("%s", temp);

	u8 skip = 0;
	while( *str ) {
		if( *str == '%' )
		switch(  *(  str+1  )  ) {
		case '%':
			printf(  "%%"  );
			skip = 1;
			break;

		case 'c':
			printf(  "%c", *(  (  string  ) argp  )  );
			argp += 1;
			skip = 1;
			break;

		case 's':
			printf(  "%s", (  string  ) argp  );
			argp += 1;
			break;

		} else {
			printf(  "%c", *str  );
		}

		str += 1 + skip;
		skip = 0;
	}
}
