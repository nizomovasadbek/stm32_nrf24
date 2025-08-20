/*
 * receive_poll.c
 *
 *  Created on: Aug 18, 2025
 *      Author: Asato
 */

#include "util/util.h"
#include "tasks/receive_poll.h"
#include "cmsis_os.h"
#include "type.h"
#include "string.h"
#include "NRF24.h"

extern osMessageQueueId_t change_permitHandle;
extern osMutexId_t dataMutexHandle;

void rx_poll(  Dummy_t* data  ) {

	osMutexAcquire(  dataMutexHandle, osWaitForever  );

	memset(  data, 0, 32  );

	osMutexRelease(  dataMutexHandle  );

	u8 sig = 0;

	while(  1  ) {

		if(  osMessageQueueGet(  change_permitHandle, &sig, 0, osWaitForever  )  ==  osOK &&  sig  ) {

			taskENTER_CRITICAL();

			nrf24_listen();

			while(  !nrf24_data_available()  );

			nrf24_receive(  (  uint8_t*  )  data, 32  );
			nrf24_stop_listen();

			taskEXIT_CRITICAL();

			if(  validate_checksum(  data  )  ==  VALIDATE_FAIL  ) {
				data = null ;
			}

		}

	}

}
