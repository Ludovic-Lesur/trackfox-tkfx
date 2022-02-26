/*
 * tim.h
 *
 *  Created on: 3 july 2021
 *      Author: Ludo
 */

#ifndef TIM_H
#define TIM_H

/*** TIM functions ***/

void TIM21_init(void);
void TIM21_get_lsi_frequency(unsigned int* lsi_frequency_hz);
void TIM21_disable(void);

#endif /* TIM_H */
