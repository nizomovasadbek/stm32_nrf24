/*
 * util.h
 *
 *  Created on: Aug 4, 2025
 *      Author: Asato
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "stdint.h"

#include "protocol.h"
#include "ota.h"

#define VALIDATE_FAIL		0
#define VALIDATE_OK			1

OTAFUS uint8_t calculate_checksum( Dummy_t* packet );
OTAFUS uint8_t validate_checksum( Dummy_t* packet );

uint32_t map(  uint32_t, uint32_t, uint32_t, uint32_t, uint32_t  );

#endif /* INC_UTIL_H_ */
