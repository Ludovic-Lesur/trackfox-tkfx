/*
 * usart.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __USART_H__
#define __USART_H__

#include "mode.h"

/*** USART functions ***/

void USART2_init(void);
#ifdef ATM
void USART2_send_string(char* tx_string);
#endif

#endif /* __USART_H__ */
