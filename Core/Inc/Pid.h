/*
 * Pid.h
 *
 *  Created on: Jan 26, 2021
 *      Author: Edoardo Idà
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct {
	double cu_i_1;
	double cu_i_2;
	double ce_i_0;
	double ce_i_1;
	double ce_i_2;
}  pid_par_t;

typedef struct {
	double u[3];
	double e[3];
}  pid_var_t;

typedef struct {
	pid_var_t pid_var;
	pid_par_t pid_par;
}  pid_t;

void pid_init(pid_t* pid, double Kp, double Ki, double Kd, uint16_t control_period_ms, double N);
double pid_update(pid_t* pid, double e);

#endif /* INC_PID_H_ */
