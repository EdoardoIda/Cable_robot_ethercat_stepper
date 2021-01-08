/*
 * Loadcell.c
 *
 *  Created on: Dec 17, 2020
 *      Author: EdoOffice
 */

#include "Loadcell.h"

void loadcell_init(loadcell_t *loadcell, ADC_HandleTypeDef *hadc, uint16_t v_to_N) {
	loadcell->loadcell_par.adc_handle = hadc;
	loadcell->loadcell_par.voltage_to_tension = v_to_N;
	HAL_ADC_Start(loadcell->loadcell_par.adc_handle);
}

uint16_t loadcell_get_value(loadcell_t *loadcell) {
	uint32_t app = HAL_ADC_GetValue(loadcell->loadcell_par.adc_handle)*3300/65535;
	loadcell->loadcell_var.cable_tension = (uint16_t)app;
	return loadcell->loadcell_var.cable_tension;
}
