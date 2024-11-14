/*
 * uart_header.h
 *
 *  Created on: Feb 26, 2024
 *      Author: J.Verkade
 */

#ifndef INC_UART_HEADER_H_
#define INC_UART_HEADER_H_

#define UART_PACKAGE_SIZE 9

typedef struct UartPingPackage_t UartPingPackage_t;
struct UartPingPackage_t
{
	uint8_t type;
	uint8_t none0;
	uint8_t none1;
	uint8_t none2;
	uint8_t none3;
	uint8_t none4;
	uint8_t none5;
	uint8_t none6;
	uint8_t none7;
} __attribute__((packed));

#define UART_ACK	0xF0
#define UART_NACK	0xF1
typedef struct UartResponsePackage_t UartResponsePackage_t;
struct UartResponsePackage_t
{
	uint8_t response;
	uint8_t none0;
	uint8_t none1;
	uint8_t none2;
	uint8_t none3;
	uint8_t none4;
	uint8_t none5;
	uint8_t none6;
	uint8_t none7;
} __attribute__((packed));

#endif /* INC_UART_HEADER_H_ */
