/*
 * rtc.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef RTC_H
#define RTC_H

#include "mode.h"
#include "neom8n.h"

/*** RTC functions ***/

void RTC_Reset(void);
void RTC_Init(unsigned char* rtc_use_lse);
volatile unsigned char RTC_GetAlarmAFlag(void);
void RTC_ClearAlarmAFlag(void);

#endif /* RTC_H */
