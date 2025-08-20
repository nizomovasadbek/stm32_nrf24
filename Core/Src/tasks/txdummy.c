/*
 * txdummy.c
 *
 *  Created on: Aug 18, 2025
 *      Author: Asato
 */


#include "tasks/txdummy.h"


void transmit_dummy(  void  ) {

	Command_t cmd;
	memset(  &cmd, 0, 32  );
	cmd.type = PACKET_COMMAND;
	cmd.M1 = 2;
	cmd.M2 = 2;
	cmd.M3 = 121;
	cmd.M4 = 72;
	uint8_t pwm = 0;
	while(  1  )
	{
	  cmd.M2 = pwm;
	  taskENTER_CRITICAL();
	  transmit(  (  Dummy_t*  ) &cmd  );
	  pwm++;
	  taskEXIT_CRITICAL();
	  osDelay(  300  );
	}
}
