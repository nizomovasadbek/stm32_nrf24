/*
 * stick_read.h
 *
 *  Created on: Aug 19, 2025
 *      Author: Asato
 */

#ifndef INC_TASKS_STICK_READ_H_
#define INC_TASKS_STICK_READ_H_

#include "type.h"

typedef struct {
	u32 M1x;
} Stick_t;

void stick_adc_read(  void  );


#endif /* INC_TASKS_STICK_READ_H_ */
