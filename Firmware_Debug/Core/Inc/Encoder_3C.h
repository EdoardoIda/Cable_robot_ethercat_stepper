/*
 * Encoder_3C.h
 *
 *  Created on: Dec 11, 2020
 *      Author: EdoOffice
 */

#ifndef INC_ENCODER_3C_H_
#define INC_ENCODER_3C_H_

#include "main.h"

typedef enum
{
	ENC3_FORWARD = 1,
	ENC3_BACKWARD = -1,
} enc3c_dir_t;

typedef struct {
	volatile uint8_t previous_state;
	volatile int32_t position;
} enc3c_var_t;

typedef struct {
	GPIO_TypeDef  *PORT_CHAN_A;
	uint16_t pinA;
	GPIO_TypeDef  *PORT_CHAN_B;
	uint16_t pinB;
	GPIO_TypeDef  *PORT_CHAN_C;
	uint16_t pinC;
	uint8_t bitshiftA;
	uint8_t bitshiftB;
	uint8_t bitshiftC;
	enc3c_dir_t dir;
	uint32_t count_per_rev;
} enc3c_par_t;

typedef struct {
	enc3c_par_t enc3c_par;
	enc3c_var_t enc3c_var;
} enc3c_t;

void encoder_init(enc3c_t * enc3c, GPIO_TypeDef *GPIOxA, uint16_t GPIO_PinA, GPIO_TypeDef *GPIOxB, uint16_t GPIO_PinB,
		GPIO_TypeDef *GPIOxC, uint16_t GPIO_PinC, enc3c_dir_t rot_dir, uint32_t enc_counts);
void checkIT(enc3c_t * enc3c);
void update(enc3c_t * enc3c); //updates the position of the encoder, do not call from the main
inline int32_t get_position(enc3c_t * enc3c) { return enc3c->enc3c_var.position; }
inline double get_angle(enc3c_t * enc3c) { return ((double)enc3c->enc3c_var.position) * 360.0 / ((double)enc3c->enc3c_par.count_per_rev); }

#endif /* INC_ENCODER_3C_H_ */
