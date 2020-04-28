/*
 * rtc.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "rtc.h"

#include "at.h"
#include "exti_reg.h"
#include "mode.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "rtc_reg.h"
#include "tim.h"

/*** RTC local macros ***/

#define RTC_INIT_TIMEOUT_SECONDS	5

/*** RTC local global variables ***/

volatile unsigned char rtc_alra_flag;
volatile unsigned char rtc_alrb_flag;

/*** RTC local functions ***/

/* RTC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void RTC_IRQHandler(void) {
	// Alarm A interrupt.
	if (((RTC -> ISR) & (0b1 << 8)) != 0) {
		// Update flag
		rtc_alra_flag = 1;
		// Clear flag.
		RTC -> ISR &= ~(0b1 << 8); // ALRAF='0'.
	}
	// Alarm B interrupt.
	if (((RTC -> ISR) & (0b1 << 9)) != 0) {
		// Update flag
		rtc_alrb_flag = 1;
		// Clear flag.
		RTC -> ISR &= ~(0b1 << 9); // ALRNF='0'.
	}
	// Clear EXTI flag.
	EXTI -> PR |= (0b1 << 17);
}

/* ENTER INITIALIZATION MODE TO ENABLE RTC REGISTERS UPDATE.
 * @param:						None.
 * @return rtc_initf_success:	1 if RTC entered initialization mode, 0 otherwise.
 */
unsigned char RTC_EnterInitializationMode(void) {
	// Local variables.
	unsigned char rtc_initf_success = 1;
	// Enter key.
	RTC -> WPR = 0xCA;
	RTC -> WPR = 0x53;
	RTC -> ISR |= (0b1 << 7); // INIT='1'.
	unsigned int loop_start_time = TIM22_GetSeconds();
	while (((RTC -> ISR) & (0b1 << 6)) == 0) {
		// Wait for INITF='1' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + RTC_INIT_TIMEOUT_SECONDS)) {
			rtc_initf_success = 0;
			break;
		}
	}
	return rtc_initf_success;
}

/* EXIT INITIALIZATION MODE TO PROTECT RTC REGISTERS.
 * @param:	None.
 * @return:	None.
 */
void RTC_ExitInitializationMode(void) {
	RTC -> ISR &= ~(0b1 << 7); // INIT='0'.
}

/*** RTC functions ***/

/* RESET RTC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void RTC_Reset(void) {
	// Reset RTC peripheral.
	RCC -> CSR |= (0b1 << 19); // RTCRST='1'.
	unsigned char j = 0;
	for (j=0 ; j<100 ; j++) {
		// Poll a bit always read as '0'.
		// This is required to avoid for loop removal by compiler (size optimization for HW1.0).
		if (((RCC -> CR) & (0b1 << 24)) != 0) {
			break;
		}
	}
	RCC -> CSR &= ~(0b1 << 19); // RTCRST='0'.
}

/* INIT HARDWARE RTC PERIPHERAL.
 * @param rtc_use_lse:	RTC will be clocked on LSI if 0, on LSE otherwise.
 * @PARAM lsi_freq_hz:	Effective LSI oscillator frequency used to compute the accurate prescaler value (only if LSI is used as source).
 * @return:				None.
 */
void RTC_Init(unsigned char* rtc_use_lse, unsigned int lsi_freq_hz) {
	// Manage RTC clock source.
	if ((*rtc_use_lse) != 0) {
		// Use LSE.
		RCC -> CSR |= (0b01 << 16); // RTCSEL='01'.
	}
	else {
		// Use LSI.
		RCC -> CSR |= (0b10 << 16); // RTCSEL='10'.
	}
	// Enable RTC and register access.
	RCC -> CSR |= (0b1 << 18); // RTCEN='1'.
	// Switch to LSI if RTC failed to enter initialization mode.
	if (RTC_EnterInitializationMode() == 0) {
		RTC_Reset();
		RCC -> CSR |= (0b10 << 16); // RTCSEL='10'.
		RCC -> CSR |= (0b1 << 18); // RTCEN='1'.
		RTC_EnterInitializationMode();
		// Update flag.
		(*rtc_use_lse) = 0;
	}
	// Configure prescaler.
	if ((*rtc_use_lse) != 0) {
		// LSE frequency is 32.768kHz typical.
		RTC -> PRER = (127 << 16) | (255 << 0); // PREDIV_A=127 and PREDIV_S=255 (128*256 = 32768).
	}
	else {
		// Compute prescaler according to measured LSI frequency.
		RTC -> PRER = (127 << 16) | (((lsi_freq_hz / 128) - 1) << 0); // PREDIV_A=127 and PREDIV_S=((lsi_freq_hz/128)-1).
	}
	// Bypass shadow registers.
	RTC -> CR |= (0b1 << 5); // BYPSHAD='1'.
	// Configure alarm A to wake-up MCU every hour.
	RTC -> ALRMAR = 0; // Reset all bits.
	RTC -> ALRMAR |= (0b1 << 31) | (0b1 << 23); // Mask day and hour (only check minutes and seconds).
	//RTC -> ALRMAR |= (0b1 << 15); // Mask minutes (to wake-up every minute for debug).
	RTC -> CR |= (0b1 << 8); // Enable Alarm A.
	RTC -> CR |= (0b1 << 12); // Enable interrupt (ALRAIE='1').
	RTC -> ISR &= ~(0b1 << 8); // Clear flag.
	// Configure alarm B to wake-up every seconds (watchdog reload).
	RTC -> ALRMBR = 0;
	RTC -> ALRMBR |= (0b1 << 31) | (0b1 << 23) | (0b1 << 15) | (0b1 << 7); // Mask all fields.
	RTC -> CR |= (0b1 << 9); // Enable Alarm B.
	RTC -> CR |= (0b1 << 13); // Enable interrupt (ALRBIE='1').
	RTC -> ISR &= ~(0b1 << 9); // Clear flag.
	// Exit initialization mode.
	RTC_ExitInitializationMode();
	// Enable RTC alarm interrupt (line 17).
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	EXTI -> IMR |= (0b1 << 17); // IM17='1'.
	EXTI -> RTSR |= (0b1 << 17); // RTC interrupt requires rising edge.
	EXTI -> PR |= (0b1 << 17); // Clear flag.
}

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile unsigned char RTC_GetAlarmAFlag(void) {
	return rtc_alra_flag;
}

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile unsigned char RTC_GetAlarmBFlag(void) {
	return rtc_alrb_flag;
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_ClearAlarmAFlag(void) {
	// Clear ALARM and EXTI flags.
	RTC -> ISR &= ~(0b1 << 8); // Clear flags.
	EXTI -> PR |= (0b1 << 17);
	rtc_alra_flag = 0;
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_ClearAlarmBFlag(void) {
	// Clear ALARM and EXTI flags.
	RTC -> ISR &= ~(0b1 << 9); // Clear flags.
	EXTI -> PR |= (0b1 << 17);
	rtc_alrb_flag = 0;

}
