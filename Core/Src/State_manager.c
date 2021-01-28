/*
 * State_manager.c
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */


#include "State_manager.h"

extern state_manager_t state_machine;

void manager_poll4updates() {
	led_update_blink(&green_led);
	led_update_blink(&yellow_led);
	led_update_blink(&red_led);
}

void state_convert_input(state_manager_t * state_manager) {
	state_manager->status_request = state_manager->ethercat->BufferOut->Cust.control_word;
	motor.stepper_var.target_position = state_manager->ethercat->BufferOut->Cust.target_position;
	motor.stepper_var.target_speed = state_manager->ethercat->BufferOut->Cust.target_speed;
	motor.stepper_var.target_torque = state_manager->ethercat->BufferOut->Cust.target_torque;
}

void state_convert_output(state_manager_t * state_manager) {
	state_manager->ethercat->BufferIn->Cust.actual_position = motor.stepper_var.position;
	state_manager->ethercat->BufferIn->Cust.actual_position_aux = pulley_enc.enc3c_var.position;
	state_manager->ethercat->BufferIn->Cust.actual_speed = motor.stepper_var.speed;
	//state_manager->ethercat->BufferIn->Cust.actual_torque = motor.stepper_var.torque;
	state_manager->ethercat->BufferIn->Cust.actual_torque = loadcell_get_value(&loadcell);
	state_manager->ethercat->BufferIn->Cust.loadcell_value = loadcell_get_value(&loadcell);
	state_manager->ethercat->BufferIn->Cust.status_word = state_manager->status;
}



void state_manager_init(state_manager_t *state_manager, Easycat *ethercat, error_t *error_handler) {
	timer_init(&control_timer, CONTROL_PERIOD);
	led_init(&green_led, Green_Led_PIN_GPIO_Port, Green_Led_PIN_Pin, control_timer.now);
	led_init(&yellow_led, Yellow_Led_PIN_GPIO_Port, Yellow_Led_PIN_Pin, control_timer.now);
	led_init(&red_led, Red_Led_PIN_GPIO_Port, Red_Led_PIN_Pin, control_timer.now);
	error_manager_init(error_handler, &red_led);
	serial_init(&serial, &huart6, SERIAL_INTERRUPT);
	encoder_init(&pulley_enc, Enc_A_PIN_GPIO_Port, Enc_A_PIN_Pin, Enc_B_PIN_GPIO_Port, Enc_B_PIN_Pin,
				  Enc_Z_PIN_GPIO_Port, Enc_Z_PIN_Pin, ENC3_FORWARD, 5000);
	loadcell_init(&loadcell,&hadc3,1);
	stepper_init(&motor, &htim3, CONTROL_PERIOD,
				Endstop_up_PIN_GPIO_Port, Endstop_up_PIN_Pin,
				Endstop_down_PIN_GPIO_Port, Endstop_down_PIN_Pin,
				Enable_PIN_GPIO_Port, Enable_PIN_Pin,
				Step_PIN_GPIO_Port, Step_PIN_Pin,
				Direction_PIN_GPIO_Port, Direction_PIN_Pin,
				Alarm_PIN_GPIO_Port, Alarm_PIN_Pin,
				STEP_PER_REV, STP_CW);
	easyCat_Init(ethercat,&hspi1,Ethercat_SS_GPIO_Port,Ethercat_SS_Pin,error_handler);
	pid_init(&tension_pid, double Kp, 0.0, 0.0, CONTROL_PERIOD, DEFAULT_PID_FREQ_FILTER);
	tension_pid

	state_manager->control = CONTROL_POSITION;
	state_manager->state = STATE_INIT;
	state_manager->ethercat = ethercat;
	state_manager->error_handler = error_handler;
	state_manager->status = 0;
	state_manager->status_request = IDLE_STATE_BIT;

	state_manager->state_function[STATE_INIT] = &state_init_function;
	state_manager->state_function[STATE_IDLE] = &state_idle_function;
	state_manager->state_function[STATE_OPERATIONAL] = &state_operational_function;
	state_manager->state_function[STATE_ERROR] = &state_error_function;

	state_manager->state_transition_function[STATE_INIT] = &state_init_transition;
	state_manager->state_transition_function[STATE_IDLE] = &state_idle_transition;
	state_manager->state_transition_function[STATE_OPERATIONAL] = &state_operational_transition;
	state_manager->state_transition_function[STATE_ERROR] = &state_error_transition;

	state_manager->control_function[CONTROL_POSITION] = &control_position_function;
	state_manager->control_function[CONTROL_SPEED] = &control_speed_function;
	state_manager->control_function[CONTROL_TORQUE] = &control_torque_function;

	state_manager->state_transition_function[state_manager->state]();
	state_manager->state_function[state_manager->state]();
}

void state_init_function() {
	 if (is_alarm_on()) {
		 go_to_error();
	 } else {

	 }
}

void state_idle_function() {
	if (is_alarm_on()) {
			 go_to_error();
		 } else {

		 }
}

void manage_control_change() {
 if (state_machine.status_request && POSITION_CONTROL_BIT) {
	 state_machine.control = CONTROL_POSITION;
 } else if (state_machine.status_request && SPEED_CONTROL_BIT) {
	 state_machine.control = CONTROL_SPEED;
 } else if (state_machine.status_request && TORQUE_CONTROL_BIT) {
	 state_machine.control = CONTROL_TORQUE;
 }
}

void state_operational_function() {
	if (is_alarm_on()) {
			 go_to_error();
		 } else {
			 manage_control_change();
			 state_machine.control_function[state_machine.control]();
		 }
}

void state_error_function() {

}

void state_init_transition() {
	if ((state_machine.status_request & IDLE_STATE_BIT) == IDLE_STATE_BIT) {
				state_machine.state = STATE_IDLE;
				state_machine.status = state_machine.status | IDLE_STATE_BIT;
				stepper_disable(&motor);
				led_blink(&green_led, IDLE_BLINK_PERIOD);
			}
}

void state_idle_transition() {
	if ((state_machine.status_request & OPERATIONAL_STATE_BIT) == OPERATIONAL_STATE_BIT) {
		state_machine.state = STATE_OPERATIONAL;
		state_machine.status = ((state_machine.status ^ IDLE_STATE_BIT)| OPERATIONAL_STATE_BIT);
		stepper_enable(&motor);
		led_blink(&green_led, OPERATIONAL_BLINK_PERIOD);
	}
}

void state_operational_transition() {
	if ((state_machine.status_request & IDLE_STATE_BIT) == IDLE_STATE_BIT) {
			state_machine.state = STATE_IDLE;
			state_machine.status = ((state_machine.status ^ OPERATIONAL_STATE_BIT)| IDLE_STATE_BIT);
			led_blink(&green_led, IDLE_BLINK_PERIOD);
			stepper_disable(&motor);
		}
}

void state_error_transition() {
	if ((state_machine.status_request & ERROR_RESET_BIT) == ERROR_RESET_BIT) {
			state_machine.state = STATE_IDLE;
			state_machine.status = (state_machine.status ^ ERROR_STATE_BIT)| IDLE_STATE_BIT;
			stepper_disable(&motor);
		}
}

void control_position_function() {
	motor.stepper_var.update_flag = 1;
}

void control_speed_function() {
	motor.stepper_var.target_position = motor.stepper_var.position + (motor.stepper_var.target_speed*CONTROL_PERIOD)/1000;
	motor.stepper_var.update_flag = 1;
}

void control_torque_function() {
	uint16_t actual_tension = loadcell_get_value(&loadcell);

	motor.stepper_var.update_flag = 1;
}

void go_to_error() {
	state_machine.state = STATE_ERROR;
	switch (state_machine.state) {
	case STATE_IDLE : {
		state_machine.status = state_machine.status ^ IDLE_STATE_BIT;
		break;
	}
	case STATE_OPERATIONAL : {
		state_machine.status = state_machine.status ^ OPERATIONAL_STATE_BIT;
			break;
		}
	default : break;
	}
	state_machine.status = state_machine.status | ERROR_STATE_BIT;
	stepper_disable(&motor);
	led_blink(&green_led, ERROR_BLINK_PERIOD);
}

uint8_t is_alarm_on() {
	if (HAL_GPIO_ReadPin(motor.stepper_par.alarm_port, motor.stepper_par.alarm_pin) == GPIO_PIN_RESET) {
		return 1;
	} else {
		return 0;
	}
}
