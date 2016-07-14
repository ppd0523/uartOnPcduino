/*
 * uart.h
 *
 *  Created on: 2016. 7. 14.
 *      Author: em
 */

#ifndef UART_H_
#define UART_H_

#include <termios.h>

typedef struct UART{
	int fd;
	struct termios tio;
} UART;

int uart_init(UART* uart, const char* portName, int baudrate);
void uart_close(UART* uart);

int uartTx(UART* uart, const unsigned char* data, int size);
int uartRx(UART* uart, unsigned char* data, int size);

int setBaudrate(int baudrate);


#endif /* UART_H_ */
