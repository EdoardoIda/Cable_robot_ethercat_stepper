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
	uint8_t empty;
} loadcell_par_t;

typedef struct {
	loadcell_par_t loadcell_par;
	loadcell_var_t loadcell_var;
} loadcell_t;

void loadcell_init();

#endif /* INC_LOADCELL_H_ */
