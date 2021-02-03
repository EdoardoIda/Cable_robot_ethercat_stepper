/*
 * Encoder_3C.c
 *
 *  Created on: Dec 11, 2020
 *      Author: EdoOffice
 */

#include "Encoder_3C.h"

const int32_t ROT[16] = { 0, 1,  -1,  0, -1,  0,  0, 1, 1,  0,  0,  -1, 0,  -1, +1,  0 };

void encoder_init(enc3c_t * enc3c, GPIO_TypeDef *GPIOxA, uint16_t GPIO_PinA, GPIO_TypeDef *GPIOxB, uint16_t GPIO_PinB,
		GPIO_TypeDef *GPIOxC, uint16_t GPIO_PinC, enc3c_dir_t rot_dir, uint32_t enc_counts) {

	enc3c->enc3c_par.PORT_CHAN_A = GPIOxA;
	enc3c->enc3c_par.PORT_CHAN_B = GPIOxB;
	enc3c->enc3c_par.PORT_CHAN_C = GPIOxC;
	enc3c->enc3c_par.pinA = GPIO_PinA;
	enc3c->enc3c_par.pinB = GPIO_PinB;
	enc3c->enc3c_par.pinC = GPIO_PinC;
	enc3c->enc3c_par.count_per_rev = enc_counts;
	if (rot_dir==ENC3_FORWARD) {
		enc3c->enc3c_par.dir = 1;
	} else {
		enc3c->enc3c_par.dir = -1;
	}


	uint16_t k = 1;
	uint8_t temp;
	for (temp = 0;temp < 16;temp++) {
		if ((enc3c->enc3c_par.pinA) & k) break;
		k = k << 1;
	}
	enc3c->enc3c_par.bitshiftA = temp;

	k=1;
	for (temp = 0;temp < 16;temp++) {
		if ((enc3c->enc3c_par.pinB) & k) break;
		k = k << 1;
	}
	enc3c->enc3c_par.bitshiftB = temp;

	k=1;
	for (temp = 0;temp < 16;temp++) {
		if ((enc3c->enc3c_par.pinC) & k) break;
		k = k << 1;
	}
	enc3c->enc3c_par.bitshiftC = temp;

	uint8_t sigA = ((enc3c->enc3c_par.PORT_CHAN_A->IDR) & (enc3c->enc3c_par.pinA)) >> (enc3c->enc3c_par.bitshiftA);
	uint8_t sigB = ((enc3c->enc3c_par.PORT_CHAN_B->IDR) & (enc3c->enc3c_par.pinB)) >> (enc3c->enc3c_par.bitshiftB);
	enc3c->enc3c_var.previous_state = sigB | (sigA << 1);

	enc3c->enc3c_var.position = 0;

}

void checkIT(enc3c_t * enc3c) {
	if((__HAL_GPIO_EXTI_GET_IT(enc3c->enc3c_par.pinA) != RESET)||(__HAL_GPIO_EXTI_GET_IT(enc3c->enc3c_par.pinB) != RESET)||(__HAL_GPIO_EXTI_GET_IT(enc3c->enc3c_par.pinC) != RESET))
			{
				__HAL_GPIO_EXTI_CLEAR_IT(enc3c->enc3c_par.pinA);
				__HAL_GPIO_EXTI_CLEAR_IT(enc3c->enc3c_par.pinB);
				__HAL_GPIO_EXTI_CLEAR_IT(enc3c->enc3c_par.pinC);
				update(enc3c);
			}
}

void update(enc3c_t * enc3c) {
	uint8_t sigA = (enc3c->enc3c_par.PORT_CHAN_A->IDR & enc3c->enc3c_par.pinA) >> enc3c->enc3c_par.bitshiftA;
	uint8_t sigB = (enc3c->enc3c_par.PORT_CHAN_B->IDR & enc3c->enc3c_par.pinB) >> enc3c->enc3c_par.bitshiftB;
	uint8_t sigC = (enc3c->enc3c_par.PORT_CHAN_C->IDR & enc3c->enc3c_par.pinC) >> enc3c->enc3c_par.bitshiftC;
	uint8_t s = sigB | (sigA << 1);

	if (enc3c->enc3c_par.dir) {
		enc3c->enc3c_var.position += ROT[(enc3c->enc3c_var.previous_state << 2) | s];
	} else {
		enc3c->enc3c_var.position -= ROT[(enc3c->enc3c_var.previous_state << 2) | s];
	}
			if (sigC == 1) {
		enc3c->enc3c_var.position = 0;
	}
	enc3c->enc3c_var.previous_state = s;
}
