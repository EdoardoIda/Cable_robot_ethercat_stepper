/*
 * Error_manager.h
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */

#ifndef INC_ERROR_MANAGER_H_
#define INC_ERROR_MANAGER_H_


#include "Led.h"

typedef enum
{
	ERROR_1,
	ERROR_2,
	ERROR_3,
	NO_ERROR
} error_mode_t;

typedef enum
{
	ERR_BLINK_SLOW = 2000,
	ERR_BLINK_MEDIUM = 500,
	ERR_BLINK_FAST = 100,
	ERR_NO_BLINK = 0
} error_blink_period_t;

typedef struct {
	led_t *error_led;
	error_blink_period_t error_blink_period;
	error_mode_t error_mode;
} error_t;

void error_manager_init(error_t *error, led_t* error_led);
void error_start(error_t *error,error_mode_t error_mode);
void error_solve(error_t *error);


#endif /* INC_ERROR_MANAGER_H_ */
