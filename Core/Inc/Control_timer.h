/*
 * Control_timer.h
 *
 *  Created on: Dec 10, 2020
 *      Author: EdoOffice
 */

#ifndef INC_CONTROL_TIMER_H_
#define INC_CONTROL_TIMER_H_

#include "main.h"

typedef struct {
	volatile uint32_t* now;
	uint32_t before;
	uint16_t time_interval;
} control_timer_t;

void timer_init(control_timer_t *timer_v, uint16_t control_period_ms);
uint8_t timer_elapsed(control_timer_t *timer_v);

#endif /* INC_CONTROL_TIMER_H_ */
