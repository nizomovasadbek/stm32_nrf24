/*
 * temp_poll.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */


#include "tasks/temp_poll.h"

void temperature_poll(  void  ) {

	__attribute__((unused))  u16 temperature;

	while (  1  ) {

		temperature = read_adc_channel(  ADC_CHANNEL_TEMPSENSOR, ADC_SAMPLETIME_480CYCLES  );

		float T = (  (  float  )  temperature / 4095.0f  ) * 3.3f;
		T = (  (  T - 0.76f  ) / 0.0025f  ) + 25.0f;

		temperature = (  u16  ) T;

		osDelay(  2000  );

	}

}
