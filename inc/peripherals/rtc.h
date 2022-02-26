/*
 * rtc.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef RTC_H
#define RTC_H

/*** RTC macros ***/

// RTC wake-up timer period.
// Warning: this value must be lower than the watchdog period = 25s.
 #define RTC_WAKEUP_PERIOD_SECONDS	10

/*** RTC functions ***/

void RTC_reset(void);
void RTC_init(unsigned char* rtc_use_lse, unsigned int lsi_freq_hz);
void RTC_start_wakeup_timer(unsigned int delay_seconds);
void RTC_stop_wakeup_timer(void);
volatile unsigned char RTC_get_wakeup_timer_flag(void);
void RTC_clear_wakeup_timer_flag(void);

#endif /* RTC_H */
