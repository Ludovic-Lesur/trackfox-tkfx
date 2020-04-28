/*
 * tim.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef TIM_H
#define TIM_H

/*** TIM functions ***/

void TIM21_Init(void);
void TIM21_Start(void);
void TIM21_Stop(void);
void TIM21_Disable(void);

void TIM22_Init(void);
void TIM22_Start(void);
void TIM22_Stop(void);
void TIM22_Disable(void);
volatile unsigned int TIM22_GetSeconds(void);

#endif /* TIM_H */
