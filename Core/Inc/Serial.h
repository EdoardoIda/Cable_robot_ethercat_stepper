/*
 * Serial.h
 *
 *  Created on: Dec 10, 2020
 *      Author: EdoOffice
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "main.h"
#include <string.h>

typedef enum
{
	SERIAL_INTERRUPT,
	SERIAL_POLLING
} serial_mode_t;

typedef struct {
	serial_mode_t mode;
	uint8_t buffer_in[100];
	uint8_t buffer_out[100];
	UART_HandleTypeDef *handle;
	uint8_t message_sent;
	uint8_t message_received;
} serial_t;

void serial_init(serial_t *serial, UART_HandleTypeDef *handle, serial_mode_t working_mode);
void serial_write(serial_t *serial, uint8_t *message);
uint8_t* serial_read(serial_t *serial);

void serial_message_sent_callback(serial_t *serial);
void serial_message_received_callback(serial_t *serial);

#endif /* INC_SERIAL_H_ */
