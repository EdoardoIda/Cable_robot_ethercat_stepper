/*
 * Pid.c
 *
 *  Created on: Jan 26, 2021
 *      Author: Edoardo Idà
 */

#include "Pid.h"

void pid_init(my_pid_t *pid, double Kp, double Ki, double Kd, uint16_t control_period_ms, double N) {
	double Ts = (double)control_period_ms/1000.0;
	double b0 = Kp*(1.0+N*Ts)+Ki*Ts*(1+N*Ts)+Kd*N;
	double b1 = -(Kp*(2+N*Ts)+Ki*Ts+2*Kd*N);
	double b2 = Kp + Kd*N;
	double a0 = 1+N*Ts;
	double a1 = -(2+N*Ts);
	double a2 = 1;

	pid->pid_par.cu_i_1 = a1/a0;
	pid->pid_par.cu_i_2 = a2/a0;
	pid->pid_par.ce_i_0 = -b0/a0;
	pid->pid_par.ce_i_1 = -b1/a0;
	pid->pid_par.ce_i_2 = -b2/a0;

	pid->pid_var.e[0] = 0;
	pid->pid_var.e[1] = 0;
	pid->pid_var.e[2] = 0;
	pid->pid_var.u[0] = 0;
	pid->pid_var.u[1] = 0;
	pid->pid_var.u[2] = 0;

}

double pid_update(my_pid_t* pid, double e) {

	pid->pid_var.e[2] = pid->pid_var.e[1];
	pid->pid_var.e[1] = pid->pid_var.e[0];
	pid->pid_var.e[0] = e;

	pid->pid_var.u[2] = pid->pid_var.u[1];
	pid->pid_var.u[1] = pid->pid_var.u[0];
	pid->pid_var.u[0] = pid->pid_par.cu_i_1*pid->pid_var.u[1]+
						pid->pid_par.cu_i_2*pid->pid_var.u[2]+
						pid->pid_par.ce_i_0*pid->pid_var.e[0]+
						pid->pid_par.ce_i_1*pid->pid_var.e[1]+
						pid->pid_par.ce_i_2*pid->pid_var.e[2];
	return pid->pid_var.u[0];
}
