/*
 * lptim.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "lptim.h"

#include "exti.h"
#include "lptim_reg.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** LPTIM local macros ***/

#define LPTIM_TIMEOUT_COUNT		1000
#define LPTIM_DELAY_MS_MIN		4
#define LPTIM_DELAY_MS_MAX		255000

/*** LPTIM local global variables ***/

static unsigned int lptim_clock_frequency_hz = 0;

/*** LPTIM local functions ***/

/* LPTIM INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_IRQHandler(void) {
	// Clear all flags.
	LPTIM1 -> ICR |= (0b1111111 << 0);
}

/*** LPTIM functions ***/

/* INIT LPTIM FOR DELAY OPERATION.
 * @param lptim1_use_lsi:	Use LSE as clock source if non 0, LSI otherwise.
 * @return:					None.
 */
void LPTIM1_Init(unsigned char lptim1_use_lse) {
	// Disable peripheral.
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
	// Enable peripheral clock.
	RCC -> CCIPR &= ~(0b11 << 18); // Reset bits 18-19.
	if (lptim1_use_lse != 0) {
		RCC -> CCIPR |= (0b11 << 18); // LPTIMSEL='11' (LSE clock selected).
		lptim_clock_frequency_hz = (RCC_LSE_FREQUENCY_HZ >> 7);
	}
	else {
		RCC -> CCIPR |= (0b01 << 18); // LPTIMSEL='01' (LSI clock selected).
		lptim_clock_frequency_hz = (RCC_LSI_FREQUENCY_HZ >> 7);
	}
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.
	// Configure peripheral.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0'), needed to write CFGR.
	LPTIM1 -> CFGR &= ~(0b1 << 0);
	LPTIM1 -> CFGR |= (0b111 << 9); // Prescaler = 256.
	LPTIM1 -> CNT &= 0xFFFF0000; // Reset counter.
	// Enable LPTIM EXTI line.
	LPTIM1 -> IER |= (0b1 << 1); // ARRMIE='1'.
	EXTI_ConfigureLine(EXTI_LINE_LPTIM1, EXTI_TRIGGER_RISING_EDGE);
	// Clear all flags.
	LPTIM1 -> ICR |= (0b1111111 << 0);
}

/* ENABLE LPTIM1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Enable(void) {
	// Enable timer clock.
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.
}

/* DISABLE LPTIM1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Disable(void) {
	// Disable timer.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	LPTIM1 -> CNT &= 0xFFFF0000;
	// Clear all flags.
	LPTIM1 -> ICR |= (0b1111111 << 0);
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
}

/* DELAY FUNCTION.
 * @param delay_ms:	Number of milliseconds to wait.
 * @return:			None.
 */
void LPTIM1_DelayMilliseconds(unsigned int delay_ms) {
	// Clamp value if required.
	unsigned int local_delay_ms = delay_ms;
	if (local_delay_ms > LPTIM_DELAY_MS_MAX) {
		local_delay_ms = LPTIM_DELAY_MS_MAX;
	}
	if (local_delay_ms < LPTIM_DELAY_MS_MIN) {
		local_delay_ms = LPTIM_DELAY_MS_MIN;
	}
	// Enable timer.
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1').

	// Compute ARR value.
	LPTIM1 -> CNT &= 0xFFFF0000;
	LPTIM1 -> ARR = ((local_delay_ms * lptim_clock_frequency_hz) / (1000));
	unsigned int loop_count = 0;
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0) {
		// Wait for ARROK='1' or timeout.
		loop_count++;
		if (loop_count > LPTIM_TIMEOUT_COUNT) break;
	}
	// Start timer.
	NVIC_EnableInterrupt(IT_LPTIM1);
	LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
	// Enter stop mode.
	PWR_EnterStopMode();
	// Disable timer.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	NVIC_DisableInterrupt(IT_LPTIM1);
}
