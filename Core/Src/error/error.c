/*
 * error.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */


#include "error/error.h"

static u8 errors;

void error_poll(  void  ) {

	while(  1  ) {

		if(  !errors  )   continue;

		// TODO: Error handle

	}

}


void error_add(  u8 error_type  ) {

}


void _pulse(  u32 times,  u32 delay  ) {

	for(  u32  i  =  0;  i  <  times  *  2;  i++) {

		HAL_GPIO_TogglePin(  ERROR_LED_PORT, ERROR_LED_PIN  );
		osDelay(  delay  );

	}

	HAL_GPIO_WritePin(  ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_RESET  );

}
