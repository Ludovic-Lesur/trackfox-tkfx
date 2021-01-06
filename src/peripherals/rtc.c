/*
 * rtc.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "rtc.h"

#include "at.h"
#include "exti.h"
#include "exti_reg.h"
#include "mode.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "rtc_reg.h"

/*** RTC local macros ***/

#define RTC_INIT_TIMEOUT_COUNT		1000
#define RTC_WAKEUP_TIMER_DELAY_MAX	65536

/*** RTC local global variables ***/

static volatile unsigned char rtc_wakeup_timer_flag = 0;

/*** RTC local functions ***/

/* RTC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void RTC_IRQHandler(void) {
	// Wake-up timer interrupt.
	if (((RTC -> ISR) & (0b1 << 10)) != 0) {
		// Update flag
		rtc_wakeup_timer_flag = 1;
		// Clear flag.
		RTC -> ISR &= ~(0b1 << 10); // WUTF='0'.
	}
	// Clear EXTI flag.
	EXTI -> PR |= (0b1 << EXTI_LINE_RTC_WAKEUP_TIMER);
}

/* ENTER INITIALIZATION MODE TO ENABLE RTC REGISTERS UPDATE.
 * @param:						None.
 * @return rtc_initf_success:	1 if RTC entered initialization mode, 0 otherwise.
 */
static unsigned char RTC_EnterInitializationMode(void) {
	// Local variables.
	unsigned char rtc_initf_success = 1;
	// Enter key.
	RTC -> WPR = 0xCA;
	RTC -> WPR = 0x53;
	RTC -> ISR |= (0b1 << 7); // INIT='1'.
	unsigned int loop_count = 0;
	while (((RTC -> ISR) & (0b1 << 6)) == 0) {
		// Wait for INITF='1' or timeout.
		if (loop_count > RTC_INIT_TIMEOUT_COUNT) {
			rtc_initf_success = 0;
			break;
		}
		loop_count++;
	}
	return rtc_initf_success;
}

/* EXIT INITIALIZATION MODE TO PROTECT RTC REGISTERS.
 * @param:	None.
 * @return:	None.
 */
static void RTC_ExitInitializationMode(void) {
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
 * @return:				None.
 */
void RTC_Init(unsigned char* rtc_use_lse) {
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
		RTC -> PRER = (127 << 16) | (296 << 0); // PREDIV_A=127 and PREDIV_S=296 (128*295 = 38000).
	}
	// Bypass shadow registers.
	RTC -> CR |= (0b1 << 5); // BYPSHAD='1'.
	// Configure wake-up timer.
	RTC -> CR &= ~(0b1 << 10); // Disable wake-up timer.
	RTC -> CR &= ~(0b111 << 0);
	RTC -> CR |= (0b100 << 0); // Wake-up timer clocked by RTC clock (1Hz).
	RTC -> CR |= (0b1 << 14); // Enable wake-up timer interrupt.
	RTC_ExitInitializationMode();
	// Enable wake-up timer interrupt (line 17).
	EXTI_ConfigureLine(EXTI_LINE_RTC_WAKEUP_TIMER, EXTI_TRIGGER_RISING_EDGE);
}

void RTC_StartWakeUpTimer(unsigned int delay_seconds) {
	// Clamp parameter.
	unsigned int local_delay_seconds = delay_seconds;
	if (local_delay_seconds > RTC_WAKEUP_TIMER_DELAY_MAX) {
		local_delay_seconds = RTC_WAKEUP_TIMER_DELAY_MAX;
	}
	// Enable RTC and register access.
	RTC_EnterInitializationMode();
	// Configure wake-up timer.
	RTC -> CR &= ~(0b1 << 10); // Disable wake-up timer.
	RTC -> WUTR = (local_delay_seconds - 1);
	// Clear flags.
	RTC -> ISR &= ~(0b1 << 10); // WUTF='0'.
	EXTI -> PR |= (0b1 << EXTI_LINE_RTC_WAKEUP_TIMER);
	// Start timer.
	RTC -> CR |= (0b1 << 10); // Enable wake-up timer.
	RTC_ExitInitializationMode();
	// Enable interrupt.
	NVIC_EnableInterrupt(IT_RTC);
}

void RTC_StopWakeUpTimer(void) {
	// Enable RTC and register access.
	RTC_EnterInitializationMode();
	RTC -> CR &= ~(0b1 << 10); // Disable wake-up timer.
	RTC_ExitInitializationMode();
	// Disable interrupt.
	NVIC_DisableInterrupt(IT_RTC);
}

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile unsigned char RTC_GetWakeUpTimerFlag(void) {
	return rtc_wakeup_timer_flag;
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_ClearWakeUpTimerFlag(void) {
	// Clear ALARM and EXTI flags.
	rtc_wakeup_timer_flag = 0;
}
