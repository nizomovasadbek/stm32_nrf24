/*
 * tx_command.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */


#include "tasks/tx_command.h"
#include "tasks/stick_read.h"
#include "util/util.h"

#include "cmsis_os.h"
#include "string.h"
#include "transmitter.h"
#include "stdio.h"

extern osMessageQueueId_t motor_xHandle;

Command_t cmd;

void txcommand_poll(  void  ) {

	Stick_t stick;

	memset(  &stick,  0,  sizeof(  Stick_t  )  );
	memset(  &cmd,  0,  32  );

	cmd.type  =  PACKET_COMMAND;

	while(  osMessageQueueGet(  motor_xHandle,  &stick,  0,  osWaitForever  )  ==  osOK  ) {

		cmd.M1  =  (  u8  )  map(  stick.M1x, 1258, 3480, 0, 255  );

		//TODO: map and send all sticks

		taskENTER_CRITICAL();

		transmit(  (  Dummy_t*  ) &cmd  );

		taskEXIT_CRITICAL();

		osDelay(  20  );

	}

}
