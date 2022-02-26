/*
 * usart.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef USART_H
#define USART_H

#include "mode.h"

/*** USART functions ***/

void USART2_init(void);
#ifdef ATM
void USART2_send_string(char* tx_string);
#endif

#endif /* USART_H */
