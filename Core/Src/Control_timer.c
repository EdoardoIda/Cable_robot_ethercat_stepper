/*
 * Control_timer.c
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */


#include "Control_timer.h"

extern __IO uint32_t uwTick;

void timer_init(control_timer_t *timer_v, uint16_t control_period_ms) {
	timer_v->now = &uwTick;
	timer_v->before = *(timer_v->now);
	timer_v->time_interval = control_period_ms;
}

uint8_t timer_elapsed(control_timer_t *timer_v) {
	if (*(timer_v->now)-timer_v->before>=timer_v->time_interval) {
		timer_v->before = *(timer_v->now);
		return 1;
	} else {
		return 0;
	}
}
