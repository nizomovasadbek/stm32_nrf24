/*
 * tx_command.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */


#include "tasks/tx_command.h"
#include "util/util.h"

#include "cmsis_os.h"
#include "string.h"
#include "transmitter.h"
#include "stdio.h"
#include "error/error.h"
#include "adc.h"

extern ADCValues_t adc;

Command_t cmd;

void txcommand_poll(  void  ) {

	memset(  &cmd,  0,  32  );

	cmd.type  =  PACKET_COMMAND;

	while(  1  ) {

		cmd.M1  =  (  u8  )  map(  adc.M1, 1258, 3480, 0, 255  );
		cmd.M2 = (  u8  ) 127;
		cmd.M3 = (  u8  ) 127;
		cmd.M4 = (  u8  ) 127;
		cmd.M5 = (  u8  ) 127;
		cmd.M6 = (  u8  ) 127;
		cmd.M7 = (  u8  ) 127;
		cmd.M8 = (  u8  ) 127;
		cmd.M9 = (  u8  ) 127;

		printf(  "Motor PWM: %d\r\n", cmd.M1  );

		//TODO: map and send all sticks

		taskENTER_CRITICAL();

		transmit(  (  Dummy_t*  ) &cmd  );

		taskEXIT_CRITICAL();

		osDelay(  20  );

	}

}
