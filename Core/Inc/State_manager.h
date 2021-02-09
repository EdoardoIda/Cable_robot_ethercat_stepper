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

#define NUMBER_OF_STATES 4
#define NUMBER_OF_CONTROL 3

#define IDLE_STATE_BIT 0b00000001 //shared bit between status and control words
#define OPERATIONAL_STATE_BIT 0b00000010
#define ERROR_STATE_BIT 0b00000100
#define POSITION_CONTROL_BIT 0b00001000
#define SPEED_CONTROL_BIT 0b00010000
#define TORQUE_CONTROL_BIT 0b00100000

#define ERROR_RESET_BIT 0b01000000 // if written in control , affect status
#define TARGET_REACHED_BIT 0b1000000 // to be written in status

#define IDLE_BLINK_PERIOD 1000
#define OPERATIONAL_BLINK_PERIOD 500
#define ERROR_BLINK_PERIOD 250

#define DEFAULT_PID_FREQ_FILTER 100

#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#include "Led.h"
#include "Control_timer.h"
#include "Serial.h"
#include "Encoder_3C.h"
#include "StepperRT.h"
#include "Loadcell.h"
#include "Easycat.h"
#include "Error_manager.h"
#include "pid.h"

control_timer_t control_timer;
led_t green_led,
	  yellow_led,
	  red_led;
serial_t serial;
enc3c_t pulley_enc;
stepperRT_t motor;
loadcell_t loadcell;
my_pid_t tension_pid;

typedef enum
{
	STATE_INIT,
	STATE_IDLE,
	STATE_OPERATIONAL,
	STATE_ERROR,
} state_t;

typedef enum
{
	CONTROL_POSITION,
	CONTROL_SPEED,
	CONTROL_TORQUE
} control_t;

typedef struct
{
	int32_t position;
	int32_t speed;
	int16_t torque;
} target_t;

typedef void (*fun_pointer_t)();

typedef struct {
	state_t state;
	target_t target;
	control_t control;
	uint16_t status;
	uint16_t status_request;
	fun_pointer_t state_transition_function[NUMBER_OF_STATES];
	fun_pointer_t control_transition_function[NUMBER_OF_CONTROL];
	fun_pointer_t state_function[NUMBER_OF_STATES];
	fun_pointer_t control_function[NUMBER_OF_CONTROL];
	Easycat *ethercat;
	error_t *error_handler;
} state_manager_t;

void manager_poll4updates();
inline uint8_t manager_timer_elapsed() {return timer_elapsed(&control_timer);}
void state_manager_init(state_manager_t *state_manager, Easycat *ethercat, error_t *error_handler);
void state_convert_input(state_manager_t * state_manager);
void state_convert_output(state_manager_t * state_manager);

void state_init_function();
void state_idle_function();
void state_operational_function();
void state_error_function();

void state_init_transition();
void state_idle_transition();
void state_operational_transition();
void state_error_transition();

void control_position_transition();
void control_speed_transition();
void control_torque_transition();

void control_position_function();
void control_speed_function();
void control_torque_function();

void go_to_error();
uint8_t is_alarm_on();



#endif /* INC_STATE_MANAGER_H_ */
