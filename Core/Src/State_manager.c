/*
 * State_manager.c
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */


#include "State_manager.h"







void manager_poll4updates() {
	led_update_blink(&green_led);
	led_update_blink(&yellow_led);
	led_update_blink(&red_led);
}
