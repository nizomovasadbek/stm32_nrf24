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

extern ADC_HandleTypeDef hadc1;

void stick_adc_read(  void  ) {

	u32 val = 0;

	while(  1  ) {

		HAL_ADC_Start(  &hadc1  );
		HAL_ADC_PollForConversion(  &hadc1, 100  );

		val = HAL_ADC_GetValue(  &hadc1  );
		printf(  "%u\r\n", val  );


		osDelay(  800  );

	}



}
