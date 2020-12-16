/*
 * Led.c
 *
 *  Created on: Dec 10, 2020
 *      Author: EdoOffice
 */
#include "led.h"

void led_init(led_t* led, GPIO_TypeDef *led_port, uint16_t led_pin, volatile uint32_t* timer_var) {

	led->led_par.led_port = led_port;
	led->led_par.led_pin = led_pin;
	led->led_var.current_time = timer_var;
	led->led_var.last_update_time = *timer_var;
	led->led_var.blink_period = 0;
	led->led_var.state = LED_OFF;
}

void led_on(led_t* led) {
	led->led_par.led_port->BSRR = led->led_par.led_pin;
	led->led_var.state = LED_ON;
}

void led_off(led_t* led) {
	led->led_par.led_port->BSRR = (uint32_t)led->led_par.led_pin << (16U);
	led->led_var.state = LED_OFF;
}

void led_blink(led_t* led, uint16_t period) {
	led->led_par.led_port->BSRR = (uint32_t) led->led_par.led_pin << (16U);
	led->led_var.blink_period = period/2;
	led->led_var.last_update_time = *(led->led_var.current_time);
	led->led_var.state = LED_BLINKING;
}

void led_update_blink(led_t* led) {
	if (*(led->led_var.current_time) - led->led_var.last_update_time >= led->led_var.blink_period) {
		led->led_var.last_update_time = *(led->led_var.current_time);
	if ((led->led_par.led_port->ODR & led->led_par.led_pin) == led->led_par.led_pin) {
		led->led_par.led_port->BSRR = (uint32_t)led->led_par.led_pin << (16U);
	}
	else {
		led->led_par.led_port->BSRR = led->led_par.led_pin;
	}
	}
}

