/*
 * error.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */


#include "error/error.h"

static u8 errors;
static u8 fault_list[] = {

		0, // READYOK
		1, //ERROR_QUEUEFULL
		2, //ERROR_TRANSMIT
		3, //ERROR_CHECKSUM
		4, //ERROR_HARDFAULT

};

void error_poll(  void  ) {

	while(  1  ) {

		osDelay(  1000  );

		if(  !errors  )   continue;

		u8 track = 0;

		while(  errors  ) {

			track++;

			if(  errors  &  0x01  ) {

				_pulse(  fault_list[  track  ],  SHORT_DELAY  );

			}

			errors =  (  errors  >>  1  );

		}

	}

}


void error_add(  u8 error_type  ) {

	errors  |=  error_type;

}


void _pulse(  u32 times,  u32 delay  ) {

	for(  u32  i  =  0;  i  <  times  *  2;  i++) {

		HAL_GPIO_TogglePin(  ERROR_LED_PORT, ERROR_LED_PIN  );
		osDelay(  delay  );

	}

	HAL_GPIO_WritePin(  ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET  );

}
