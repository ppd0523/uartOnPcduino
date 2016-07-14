/*
 * uart.c
 *
 *  Created on: 2016. 7. 14.
 *      Author: em
 */

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include "uart.h"

#define SUCCESS 1
#define FAIL 0

#define DEBUG

int uart_init(UART* uart, const char* portName, int baudrate) {
#ifdef DEBUG
	printf("%s, %d\n", portName, uart->fd);
#endif

	uart->fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

	tcgetattr(uart->fd, &uart->tio);
	tcflush(uart->fd, TCIOFLUSH);

	switch (baudrate) {
	case 9600:
		cfsetispeed(&uart->tio, B9600);
		cfsetospeed(&uart->tio, B9600);
		break;
	case 19200:
		cfsetispeed(&uart->tio, B19200);
		cfsetospeed(&uart->tio, B19200);
		break;
	case 38400:
		cfsetispeed(&uart->tio, B38400);
		cfsetospeed(&uart->tio, B38400);
		break;
	case 115200:
		cfsetispeed(&uart->tio, B115200);
		cfsetospeed(&uart->tio, B115200);
		break;
	default:
		cfsetispeed(&uart->tio, B115200);
		cfsetospeed(&uart->tio, B115200);
		break;
	}



	uart->tio.c_cflag |= CLOCAL;
	uart->tio.c_cflag |= CREAD;

	uart->tio.c_cflag &= ~CSIZE; // clear frame size info
	uart->tio.c_cflag |= CS8;    // 8 bit frames
	uart->tio.c_cflag &= ~PARENB;    // no parity
	uart->tio.c_cflag &= ~CSTOPB;    // one stop bit

	tcsetattr(uart->fd, TCSANOW, &uart->tio);
	tcflush(uart->fd, TCIOFLUSH);

	return SUCCESS;
}

void uart_close(UART* uart){
#ifdef DEBUG
	puts("close");
#endif
	close(uart->fd);
}

int uartTx(UART* uart, const unsigned char* data, int size){
	ssize_t txSize = 0;

	txSize = write(uart->fd, data, (size_t)size);

	return (int)txSize;
}

int uartRx(UART* uart, unsigned char* data, int size){
	ssize_t rxSize = 0;

	rxSize = read(uart->fd, (void*)data, (size_t)size);
	if( rxSize >= 0)
		return rxSize;
	else
		return FAIL;
}

int SetBaudrate(int baudrate) {

return SUCCESS;
}
