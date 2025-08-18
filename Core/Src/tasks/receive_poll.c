/*
 * receive_poll.c
 *
 *  Created on: Aug 18, 2025
 *      Author: Asato
 */


#include "tasks/receive_poll.h"
#include "cmsis_os.h"
#include "type.h"

extern osMessageQueueId_t change_permitHandle;

void rx_poll(  Dummy_t* data  ) {

	memset(  data, 0, 32  );
	u8 sig = 0;

	while(  1  ) {

		if(  osMessageQueueGet(  change_permitHandle, &sig, 0, osWaitForever  ) == osOk &&  sig  ) {

			taskENTER_CRITICAL();

			nrf24_listen();
			nrf24_receive(  data, 32  );
			nrf24_stop_listen();

			taskEXIT_CRITICAL();

		}

	}

}
