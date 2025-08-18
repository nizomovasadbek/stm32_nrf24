/*
 * blinkled.c
 *
 *  Created on: Aug 18, 2025
 *      Author: Asato
 */


#include "tasks/blinkled.h"

void blink_led(  void  ) {

	while(  1  ) {
		HAL_GPIO_TogglePin(  USER_LED_GPIO_Port, USER_LED_Pin  );
		osDelay(  1000  );
	}

}
