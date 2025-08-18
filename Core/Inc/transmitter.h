/*
 * transmitter.h
 *
 *  Created on: Aug 4, 2025
 *      Author: Asato
 */

#ifndef INC_TRANSMITTER_H_
#define INC_TRANSMITTER_H_

#include "stdint.h"
#include "protocol.h"

void transmit( Dummy_t* );
void transmit_text( const char* str );

#endif /* INC_TRANSMITTER_H_ */
