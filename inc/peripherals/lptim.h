/*
 * lptim.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef LPTIM_H
#define LPTIM_H

/*** LPTIM functions ***/

void LPTIM1_init(unsigned int lsi_freq_hz);
void LPTIM1_enable(void);
void LPTIM1_disable(void);
void LPTIM1_delay_milliseconds(unsigned int delay_ms, unsigned char stop_mode);

#endif /* LPTIM_H */
