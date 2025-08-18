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
	cmd.direction = 2;
	cmd.motor_address = 2;
	cmd.main_motor_X = 121;
	cmd.main_motor_Y = 72;
	uint8_t pwm = 0;
	while(  1  )
	{
	  cmd.motor_pwm = pwm;
	  taskENTER_CRITICAL();
	  transmit(  (  Dummy_t*  ) &cmd  );
	  pwm++;
	  taskEXIT_CRITICAL();
	  osDelay(  300  );
	}
}
