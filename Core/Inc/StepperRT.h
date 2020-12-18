/*
 * StepperRT.h
 *
 *  Created on: Dec 14, 2020
 *      Author: EdoOffice
 */

#ifndef INC_STEPPERRT_H_
#define INC_STEPPERRT_H_


#include "main.h"
#include <stdlib.h>

typedef enum
{
	STP_CW = 1,
	STP_CCW = -1,
} stepperRT_orientation_t;

typedef enum
{
	STP_BACKWARD = 0U,
	STP_FORWARD,
} stepperRT_dir_t;

typedef struct {
	int32_t target_position;
	int32_t position;
	int32_t target_speed;
	int32_t speed;
	int16_t target_torque;
	int16_t torque;
	stepperRT_dir_t direction;
	uint32_t us_period;
	uint32_t temp_us_period;
	uint32_t us_cur_time;
	uint32_t us_last_time;
	uint8_t update_flag;
} stepperRT_var_t;

typedef struct {

	uint16_t up_pin;
	GPIO_TypeDef* up_port;
	uint16_t down_pin;
	GPIO_TypeDef* down_port;

} endstop_t;

typedef struct {

	TIM_HandleTypeDef* motor_timer;
	uint16_t step4cycle_max;
	uint32_t us_stepper_period;
	GPIO_TypeDef* enable_port;
	uint16_t enable_pin;
	GPIO_TypeDef* step_port;
	uint16_t step_pin;
	GPIO_TypeDef* direction_port;
	uint16_t direction_pin;
	GPIO_TypeDef* alarm_port;
    uint16_t alarm_pin;
    endstop_t endstop;
    stepperRT_orientation_t motor_dir;
    double rad_to_steps;
} stepperRT_par_t;

typedef struct {
	stepperRT_var_t stepper_var;
	stepperRT_par_t stepper_par;
} stepperRT_t;

void stepper_init(stepperRT_t* stepper, TIM_HandleTypeDef* timer, uint16_t control_period,
		GPIO_TypeDef *endstop_up_port, uint16_t endstop_up_pin,
		GPIO_TypeDef *endstop_down_port, uint16_t endstop_down_pin,
		GPIO_TypeDef *enable_port, uint16_t enable_pin,
		GPIO_TypeDef *step_port, uint16_t step_pin,
		GPIO_TypeDef *direction_port, uint16_t direction_pin,
		GPIO_TypeDef *alarm_port, uint16_t alarm_pin,
		uint16_t step_per_rev, stepperRT_dir_t direction);

inline void stepper_disable(stepperRT_t* stepper){stepper->stepper_par.enable_port->BSRR = stepper->stepper_par.enable_pin;}
inline void stepper_enable(stepperRT_t* stepper){stepper->stepper_par.enable_port->BSRR = (uint32_t) stepper->stepper_par.enable_pin << (16U);}
inline void stepper_set_update_flag(stepperRT_t* stepper){stepper->stepper_var.update_flag = 1;}
void stepper_timer_update(stepperRT_t* stepper);
void stepper_update(stepperRT_t* stepper);
inline void stepper_set_home(stepperRT_t* stepper){stepper->stepper_var.position=0;};
inline void stepper_set_dir(stepperRT_t* stepper){(stepper->stepper_var.direction == STP_FORWARD) ? (stepper->stepper_par.direction_port->BSRR = (uint32_t)stepper->stepper_par.direction_pin) : (stepper->stepper_par.direction_port->BSRR = (uint32_t)stepper->stepper_par.direction_pin<<(16U));}
inline void stepper_set_period(stepperRT_t* stepper,uint16_t delta) {(delta>0) ? (stepper->stepper_var.us_period = stepper->stepper_par.step4cycle_max/(2*delta)) : (stepper->stepper_var.us_period =0);};
#endif /* INC_STEPPERRT_H_ */
