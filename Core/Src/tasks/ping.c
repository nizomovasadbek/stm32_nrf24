/*
 * ping.c
 *
 *  Created on: Aug 18, 2025
 *      Author: Asato
 */


#include "tasks/ping.h"
#include "type.h"
#include "protocol.h"
#include "transmitter.h"
#include "cmsis_os.h"

void transmit_ping(  void  ) {

	Ping_t p;
	memset(  &p, 0, 32  );

	p.ack = PING_SEND;
	p.type = PACKET_PING;

	while(  1  ) {

		taskENTER_CRITICAL();

		transmit(  (  Dummy_t*  ) &p  );

		taskEXIT_CRITICAL();

		osDelay(  100  );

	}


}
