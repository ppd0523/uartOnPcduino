/*
 * main.c
 *
 *  Created on: 2016. 7. 14.
 *      Author: em
 */

#include <stdio.h>

#include "uart.h"

UART uart;

int main(void){
	uart_init(&uart, (char*)"/dev/ttyUSB0", 115200);

	uartTx(&uart, (unsigned char*)"hello\n", 6);

	uart_close(&uart);

	return 0;
}
