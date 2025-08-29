/*
 * adc.h
 *
 *  Created on: Aug 20, 2025
 *      Author: Asato
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "type.h"

typedef struct {

	u16 M1;
	u16 temp;

} ADCValues_t;

u16 read_adc_channel(  u32 channel, u32 sample_rate  );


#endif /* INC_ADC_H_ */
