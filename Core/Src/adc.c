/*
 * adc.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */


#include "adc.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

__attribute__((unused)) u16 read_adc_channel(  u32 channel, u32 sample_rate  ) {

	ADC_ChannelConfTypeDef sConfig;

	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = sample_rate;

	HAL_ADC_ConfigChannel(  &hadc1, &sConfig  );

	HAL_ADC_Start(  &hadc1  );
	HAL_ADC_PollForConversion(  &hadc1, HAL_MAX_DELAY  );

	u16 val = HAL_ADC_GetValue(  &hadc1  );

	HAL_ADC_Stop(  &hadc1  );


	return val;

}
