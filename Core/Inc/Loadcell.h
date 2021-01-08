/*
 * Loadcell.h
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */

#ifndef INC_LOADCELL_H_
#define INC_LOADCELL_H_

#include "main.h"

typedef struct {
	uint16_t cable_tension;
} loadcell_var_t;

typedef struct {
	ADC_HandleTypeDef* adc_handle;
	uint16_t voltage_to_tension;
} loadcell_par_t;

typedef struct {
	loadcell_par_t loadcell_par;
	loadcell_var_t loadcell_var;
} loadcell_t;

void loadcell_init(loadcell_t *loadcell, ADC_HandleTypeDef *hadc, uint16_t v_to_N);
uint16_t loadcell_get_value(loadcell_t *loadcell);

#endif /* INC_LOADCELL_H_ */
