/*
 * Led.h
 *
 *  Created on: Dec 10, 2020
 *      Author: EdoOffice
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_

#include "main.h"

typedef enum
{
	LED_OFF,
	LED_ON,
	LED_BLINKING
} led_state_t;

typedef struct {
	led_state_t state;
	uint16_t blink_period;
	uint32_t last_update_time;
	volatile uint32_t* current_time;
} led_var_t;

typedef struct {
	GPIO_TypeDef  *led_port;
	uint16_t led_pin;
} led_par_t;

typedef struct {
	led_par_t led_par;
	led_var_t led_var;
} led_t;


void led_init(led_t* led, GPIO_TypeDef *led_port, uint16_t led_pin, volatile uint32_t* timer_var);
void led_on(led_t* led);
void led_off(led_t* led);
void led_blink(led_t* led, uint16_t period);
void led_update_blink(led_t* led);

#endif /* SRC_LED_H_ */
