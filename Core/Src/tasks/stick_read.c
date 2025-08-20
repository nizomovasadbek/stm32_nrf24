/*
 * stick_read.c
 *
 *  Created on: Aug 19, 2025
 *      Author: Asato
 */


#include "tasks/stick_read.h"

#include "main.h"
#include "cmsis_os.h"
#include "type.h"
#include "stdio.h"
#include "adc.h"

#include "error/error.h"

extern ADC_HandleTypeDef hadc1;
extern osMessageQueueId_t motor_xHandle;

Stick_t s;
//TODO: Read all adc channels and pack into one struct
void stick_adc_read(  void  ) {

	while(  1  ) {

		s.M1x = read_adc_channel(  ADC_CHANNEL_8, ADC_SAMPLETIME_15CYCLES  );
		if(  osMessageQueuePut(  motor_xHandle,  &s,  0,  1  ) != osOK  ) {

			error_add(  ERROR_QUEUEFULL  );

		}

		osDelay(  20  );

	}



}
