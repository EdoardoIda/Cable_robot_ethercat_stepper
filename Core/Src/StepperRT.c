/*
 * StepperRT.c
 *
 *  Created on: Dec 14, 2020
 *      Author: EdoOffice
 */

#include "StepperRT.h"
#define CLOCK_FREQ 240000000U
#define US4S 1000000U

void stepper_init(stepperRT_t* stepper, TIM_HandleTypeDef* timer, uint16_t control_period,
		GPIO_TypeDef *endstop_up_port, uint16_t endstop_up_pin,
		GPIO_TypeDef *endstop_down_port, uint16_t endstop_down_pin,
		GPIO_TypeDef *enable_port, uint16_t enable_pin, GPIO_TypeDef *step_port,
		uint16_t step_pin, GPIO_TypeDef *direction_port, uint16_t direction_pin,
		GPIO_TypeDef *alarm_port, uint16_t alarm_pin, uint16_t step_per_rev,
		stepperRT_dir_t direction) {

	stepper->stepper_par.motor_timer = timer;
	stepper->stepper_par.us_stepper_period = ((timer->Init.Period+1)*((uint32_t) US4S))/((uint32_t)CLOCK_FREQ/(timer->Init.Prescaler+1));
	stepper->stepper_par.step4cycle_max = control_period*1000/(2*stepper->stepper_par.us_stepper_period);
	stepper->stepper_par.enable_port = enable_port;
	stepper->stepper_par.enable_pin = enable_pin;
	stepper->stepper_par.step_port = step_port;
	stepper->stepper_par.step_pin = step_pin;
	stepper->stepper_par.direction_port = direction_port;
	stepper->stepper_par.direction_pin = direction_pin;
	stepper->stepper_par.alarm_port = alarm_port;
	stepper->stepper_par.alarm_pin = alarm_pin;
	stepper->stepper_par.endstop.down_port = endstop_down_port;
	stepper->stepper_par.endstop.down_pin = endstop_down_pin;
	stepper->stepper_par.endstop.up_port = endstop_up_port;
	stepper->stepper_par.endstop.up_pin = endstop_up_pin;
	stepper->stepper_par.endstop.is_hit = NOT_HIT;
	stepper->stepper_par.motor_dir = direction;
	stepper->stepper_par.rad_to_steps = 2.0 * M_PI / ((double) step_per_rev);

	stepper->stepper_var.position = 0;
	stepper->stepper_var.speed = 0;
	stepper->stepper_var.torque = 0;
	stepper->stepper_var.target_position = 0;
	stepper->stepper_var.target_speed = 0;
	stepper->stepper_var.target_torque = 0;
	stepper->stepper_var.direction = STP_FORWARD;
	stepper->stepper_var.us_period = 0;
	stepper->stepper_var.us_cur_time = 0;
	stepper->stepper_var.us_last_time = 0;
	stepper->stepper_var.update_flag = 0;

	stepper_disable(stepper);
	endstop_update(stepper);
	HAL_TIM_Base_Start_IT(stepper->stepper_par.motor_timer);

}

void stepper_timer_update(stepperRT_t* stepper) {
	if (__HAL_TIM_GET_FLAG(stepper->stepper_par.motor_timer, TIM_FLAG_UPDATE) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(stepper->stepper_par.motor_timer, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(stepper->stepper_par.motor_timer, TIM_IT_UPDATE);
			stepper->stepper_var.us_cur_time += stepper->stepper_par.us_stepper_period;
			(stepper->stepper_var.update_flag) ? stepper_update(stepper) : (0);
			if (stepper->stepper_var.us_cur_time - stepper->stepper_var.us_last_time > stepper->stepper_var.us_period) {
				stepper->stepper_var.us_last_time = stepper->stepper_var.us_cur_time;
				if (stepper->stepper_var.us_period != 0) {
				if ((stepper->stepper_par.step_port->ODR & stepper->stepper_par.step_pin) == stepper->stepper_par.step_pin) {
					stepper->stepper_par.step_port->BSRR = (uint32_t) stepper->stepper_par.step_pin << (16U) ;
				} else {
					stepper->stepper_par.step_port->BSRR = stepper->stepper_par.step_pin;
					(stepper->stepper_var.direction == STP_FORWARD) ? (stepper->stepper_var.position++) : (stepper->stepper_var.position--);
					(stepper->stepper_var.target_position==stepper->stepper_var.position) ? (stepper->stepper_var.us_period = 0) : (0);
				}
				}
			}
		}
	}
}

void endstop_update(stepperRT_t* stepper) {

	__HAL_GPIO_EXTI_CLEAR_IT(stepper->stepper_par.endstop.down_pin);
	__HAL_GPIO_EXTI_CLEAR_IT(stepper->stepper_par.endstop.up_pin);
 if ((HAL_GPIO_ReadPin(stepper->stepper_par.endstop.down_port, stepper->stepper_par.endstop.down_pin) == GPIO_PIN_SET) || (HAL_GPIO_ReadPin(stepper->stepper_par.endstop.up_port, stepper->stepper_par.endstop.up_pin) == GPIO_PIN_SET)) {
	 if (HAL_GPIO_ReadPin(stepper->stepper_par.endstop.down_port, stepper->stepper_par.endstop.down_pin) == GPIO_PIN_SET) {
		 stepper->stepper_par.endstop.is_hit = HIT_DOWN;
	 } else {
		 stepper->stepper_par.endstop.is_hit = HIT_UP;
	 }
	 stepper->stepper_var.us_period = 0;
 } else {
	 stepper->stepper_par.endstop.is_hit = NOT_HIT;
 }

}

void stepper_update(stepperRT_t* stepper) {
	stepper->stepper_var.update_flag = 0;
 int16_t delta = stepper->stepper_var.target_position-stepper->stepper_var.position;
 if (delta>=0 && stepper->stepper_par.endstop.is_hit!= HIT_UP) {
	 stepper->stepper_var.direction = STP_FORWARD;
	 stepper_set_period(stepper,delta);
 } else if (stepper->stepper_par.endstop.is_hit!= HIT_DOWN) {
	 stepper->stepper_var.direction = STP_BACKWARD;
	 stepper_set_period(stepper,-delta);
 }
 stepper_set_dir(stepper);
}
