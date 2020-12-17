/*
 * Error_manager.c
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */


#include "Error_manager.h"


void error_manager_init(error_t *error, led_t* error_led) {
	error->error_led = error_led;
	error->error_blink_period = ERR_NO_BLINK;
	error->error_mode = NO_ERROR;

}

void error_start(error_t *error,error_mode_t error_mode) {
	error->error_mode = error_mode;
	switch (error->error_mode) {
	case ERROR_1 : {
		error->error_blink_period = ERR_BLINK_SLOW;
		led_blink(error->error_led, (uint16_t) error->error_blink_period);
		break;
	}
	case ERROR_2 : {
		error->error_blink_period = ERR_BLINK_MEDIUM;
		led_blink(error->error_led, (uint16_t) error->error_blink_period);
		break;
	}
	case ERROR_3 : {
		error->error_blink_period = ERR_BLINK_FAST;
		led_blink(error->error_led, (uint16_t) error->error_blink_period);
		break;
	}
	case NO_ERROR : {
		error->error_blink_period = ERR_NO_BLINK;
		led_off(error->error_led);
		break;
	}
	}
}

void error_solve(error_t *error) {
	error_start(error, NO_ERROR);
}
