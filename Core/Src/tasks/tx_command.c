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

extern osMessageQueueId_t motor_xHandle;

void txcommand_poll(  void  ) {

	Stick_t s;
	Command_t cmd;

	memset(  &s,  0,  sizeof(  Stick_t  )  );
	memset(  &cmd,  0,  32  );

	while(  osMessageQueueGet(  motor_xHandle,  &s,  0,  osWaitForever  )  ==  osOK  ) {

		cmd.type  =  PACKET_COMMAND;
		cmd.main_motor_X  =  (  u8  )  map(  s.M1x, 1024, 4096, 0, 255  );

		taskENTER_CRITICAL();

		transmit(  (  Dummy_t*  ) &cmd  );

		taskEXIT_CRITICAL();

		osDelay(  10  );

	}

}
