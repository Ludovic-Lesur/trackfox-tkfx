/*
 * usart.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef USART_H
#define USART_H

#include "mode.h"

#ifdef ATM
/*** USART structures ***/

typedef enum {
	USART_FORMAT_BINARY,
	USART_FORMAT_HEXADECIMAL,
	USART_FORMAT_DECIMAL,
	USART_FORMAT_ASCII
} USART_Format;
#endif

/*** USART functions ***/

void USART2_Init(void);
#ifdef ATM
void USART2_SendValue(unsigned int tx_value, USART_Format format, unsigned char print_prefix);
void USART2_SendString(char* tx_string);
#endif

#endif /* USART_H */
