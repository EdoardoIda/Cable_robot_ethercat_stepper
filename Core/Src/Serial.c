/*
 * Serial.c
 *
 *  Created on: Dec 10, 2020
 *      Author: EdoOffice
 */

#include "Serial.h"

void serial_init(serial_t *serial, UART_HandleTypeDef *handle, serial_mode_t working_mode) {

	serial->mode = working_mode;
	serial->handle = handle;
	serial->message_sent = 0;
	serial->message_received = 0;

}

void serial_write(serial_t *serial, uint8_t *message) {

	serial->message_sent = 0;
	memset(serial->buffer_out, 0, sizeof(serial->buffer_out));
	strcpy((char*)&(serial->buffer_out[0]), (char*)message);
	size_t len = strlen((const char *)&(serial->buffer_out[0]));

	if (serial->mode == SERIAL_INTERRUPT) {
		HAL_UART_Transmit_IT(serial->handle, &(serial->buffer_out[0]), len);
	} else {
		HAL_UART_Transmit(serial->handle, &(serial->buffer_out[0]), len, 1);
		serial->message_sent = 1;
	}
}

uint8_t* serial_read(serial_t *serial) {

	serial->message_received = 0;
	memset(serial->buffer_in, 0, sizeof(serial->buffer_in));
	size_t len = sizeof(serial->buffer_in);

	if (serial->mode == SERIAL_INTERRUPT) {
		HAL_UART_Receive_IT(serial->handle, &(serial->buffer_in[0]), len);
	} else {
		HAL_UART_Receive(serial->handle, &(serial->buffer_out[0]), len,20);
		serial->message_received= 1;
	}

	return &(serial->buffer_in[0]);
}

void serial_message_sent_callback(serial_t *serial){
	serial->message_sent = 1;
}

void serial_message_received_callback(serial_t *serial){
	serial->message_received = 1;
}
