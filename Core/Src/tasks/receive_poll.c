/*
 * receive_poll.c
 *
 *  Created on: Aug 18, 2025
 *      Author: Asato
 */


#include "tasks/receive_poll.h"
#include "cmsis_os.h"
#include "type.h"
#include "string.h"

extern osMessageQueueId_t change_permitHandle;
extern osMutexId_t dataMutexHandle;

void rx_poll(  Dummy_t* data  ) {

	osMutexAcquire(  dataMutexHandle, osWaitForever  );

	memset(  data, 0, 32  );

	osMutexRelease(dataMutexHandle);

	u8 sig = 0;

	while(  1  ) {

		if(  osMessageQueueGet(  change_permitHandle, &sig, 0, osWaitForever  ) == osOK &&  sig  ) {

			taskENTER_CRITICAL();

			nrf24_listen();
			nrf24_receive(  data, 32  );
			nrf24_stop_listen();

			taskEXIT_CRITICAL();

		}

	}

}
