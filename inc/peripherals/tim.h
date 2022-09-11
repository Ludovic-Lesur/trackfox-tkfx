/*
 * tim.h
 *
 *  Created on: 3 july 2021
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

/*** TIM functions ***/

void TIM21_init(void);
void TIM21_get_lsi_frequency(unsigned int* lsi_frequency_hz);
void TIM21_disable(void);

#endif /* __TIM_H__ */
