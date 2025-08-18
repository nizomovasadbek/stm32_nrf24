/*
 * mode.c
 *
 *  Created on: Aug 18, 2025
 *      Author: Asato
 */


#include "util/mode.h"
#include "transmitter.h"
#include "protocol.h"
#include "string.h"
#include "cmsis_os.h"

extern osMessageQueueId_t change_permitHandle;
extern osThreadId_t nrf24_receiveHandle;

void change_request(  u8 mode  ) {

	if (  mode == CHANGE_TRANSMITTER  )  return ;

	u8 sig = 1 ;

	Change_t change ;
	memset(  &change,  0,  32  ) ;

	change.type  =  PACKET_CHANGE ;
	change.mode  =  mode ;

	taskENTER_CRITICAL();

	transmit(  (  Dummy_t*  ) &change  ) ;
	osMessageQueuePut(  change_permitHandle, &sig, 0, osWaitForever  );
	vTaskResume(  nrf24_receiveHandle  );

	taskEXIT_CRITICAL();

}
