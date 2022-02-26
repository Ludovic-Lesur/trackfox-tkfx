/*
 * pwr.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef PWR_H
#define PWR_H

/*** PWR functions ***/

void PWR_init(void);
void PWR_enter_sleep_mode(void);
void PWR_enter_low_power_sleep_mode(void);
void PWR_enter_stop_mode(void);

#endif /* PWR_H */
