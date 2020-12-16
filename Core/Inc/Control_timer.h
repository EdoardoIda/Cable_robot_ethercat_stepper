/*
 * Control_timer.h
 *
 *  Created on: Dec 10, 2020
 *      Author: EdoOffice
 */

#ifndef INC_CONTROL_TIMER_H_
#define INC_CONTROL_TIMER_H_

#include "main.h"
extern __IO uint32_t uwTick;

typedef struct {
	volatile uint32_t* now;
	uint32_t before;
	uint16_t time_interval;
} control_timer_t;

void timer_init(control_timer_t *timer_v, uint16_t control_period_ms) {
	timer_v->now = &uwTick;
	timer_v->before = *(timer_v->now);
	timer_v->time_interval = control_period_ms;
}

inline uint8_t timer_elapsed(control_timer_t *timer_v) {
	if (*(timer_v->now)-timer_v->before>=timer_v->time_interval) {
		timer_v->before = *(timer_v->now);
		return 1;
	} else {
		return 0;
	}
}

#endif /* INC_CONTROL_TIMER_H_ */
