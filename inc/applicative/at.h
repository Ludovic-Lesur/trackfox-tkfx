/*
 * at.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef AT_H
#define AT_H

#include "mode.h"

#ifdef ATM

/*** AT functions ***/

void AT_init(void);
void AT_task(void);
void AT_fill_rx_buffer(unsigned char rx_byte);
void AT_print_test_result(unsigned char status, int rssi);

#endif

#endif /* AT_H */
