/*
 * transmitter.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */


#include "transmitter.h"
#include "util/util.h"
#include "NRF24.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "logger.h"

void transmit(  Dummy_t* packet  ) {

	packet->checksum = 0;
	packet->checksum = calculate_checksum(  packet  );

	if(  nrf24_transmit(  (  uint8_t*  ) packet, 32  )  ) {

		printf(  "Transmission successful!\r\n"  );

	}


}

void transmit_text(  const char* str  ) {

	uint32_t len = strlen(  str  );
	uint32_t cycle_count = (  len / 25  ) + 1;

	Message_t msg;

	for(  uint32_t i = 0; i < cycle_count; i++  ) {

		uint32_t msg_len = len>25?25:len;
		memset(  &msg, 0, 32  );

		msg.status = msg_len  >=  25  ?  STATUS_TEXT_SENDING  :  STATUS_TEXT_EOL;
		msg.type = PACKET_TEXT;

		memcpy(  msg.payload, str, msg_len  );
		msg.checksum = calculate_checksum(  (  Dummy_t*  ) &msg  );

		len -= msg_len;
		str += msg_len;

		nrf24_transmit(  (  uint8_t*  ) &msg, 32  );
		printf(  "Text transmitted: %25s\r\n", msg.payload  );

	}
}
