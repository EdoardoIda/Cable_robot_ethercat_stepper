/*
 * State_manager.h
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */

#ifndef INC_STATE_MANAGER_H_
#define INC_STATE_MANAGER_H_

#define CONTROL_PERIOD 1 // milliseconds
#define STEP_PER_REV 8000 // also set in hardware

#include "Led.h"
#include "Control_timer.h"
#include "Serial.h"
#include "Encoder_3C.h"
#include "StepperRT.h"
#include "Loadcell.h"

control_timer_t control_timer;
led_t green_led,
	  yellow_led,
	  red_led;
serial_t serial;
enc3c_t pulley_enc;
stepperRT_t motor;
loadcell_t loadcell;

void manager_poll4updates();
inline uint8_t manager_timer_elapsed() {return timer_elapsed(&control_timer);}

#endif /* INC_STATE_MANAGER_H_ */
