/*
 * util.c
 *
 *  Created on: Aug 4, 2025
 *      Author: Asato
 */

#include "util/util.h"

OTAFUS uint8_t calculate_checksum(  Dummy_t* packet  ) {

	packet->checksum = 0;
	uint8_t* decoy = (  uint8_t*  ) packet;
	uint8_t result = 0;

	for(  uint32_t i = 0;  i < 32;  i++  ) {

		result ^= decoy[ i ];

	}

	return result;
}

OTAFUS uint8_t validate_checksum(  Dummy_t* packet  ) {

	uint8_t checksum1 = packet->checksum;
	packet->checksum = 0;
	uint8_t checksum2 = calculate_checksum(  packet  );

	if(  checksum1  ==  checksum2  ) {

		return VALIDATE_OK;

	}

	return VALIDATE_FAIL;

}

int32_t map(  int32_t value, int32_t fromMin, int32_t fromMax, int32_t toMin, int32_t toMax  ) {

	return (  value - fromMin  ) * (  toMax - toMin  ) / (  fromMax - fromMin  ) + toMin;

}
