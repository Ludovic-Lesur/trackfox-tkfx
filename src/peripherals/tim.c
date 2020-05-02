/*
 * tim.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "tim.h"

#include "mode.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"
#include "usart.h"

/*** TIM functions ***/

/* CONFIGURE TIM21 TO OVERFLOW EVERY SECOND.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.
	// Reset timer before configuration.
	TIM21 -> CR1 &= ~(0b1 << 0); // Disable TIM21 (CEN='0').
	TIM21 -> CNT &= 0xFFFF0000; // Reset counter.
	TIM21 -> SR &= 0xFFFFF9B8; // Clear all flags.
	// Configure TIM21 as master to count milliseconds and overflow every seconds.
	TIM21 -> PSC = RCC_GetSysclkKhz(); // Timer is clocked by SYSCLK (see RCC_Init() function). SYSCLK_KHZ-1 ?
	TIM21 -> ARR = 1000;
	TIM21 -> CR2 &= ~(0b111 << 4); // Reset bits 4-6.
	TIM21 -> CR2 |= (0b010 << 4); // Generate trigger on update event (MMS='010').
	// Generate event to update registers.
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
}

/* ENABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Start(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.
	// Enable TIM21 peripheral.
	TIM21 -> SR &= ~(0b1 << 0); // Clear flag (UIF='0').
	TIM21 -> CR1 |= (0b1 << 0); // Enable TIM21 (CEN='1').
}

/* STOP TIM21 COUNTER.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Stop(void) {
	// Stop TIM21.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* DISABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Disable(void) {
	// Disable TIM21 peripheral.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM21 -> CNT = 0;
	// Clear all flags.
	TIM21 -> SR &= 0xFFFFF9B8;
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 2); // TIM21EN='0'.
}

/* CONFIGURE TIM22 TO COUNT SECONDS SINCE START-UP.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 5); // TIM22EN='1'.
	// Reset timer before configuration.
	TIM22 -> CR1 &= ~(0b1 << 0); // Disable TIM22 (CEN='0').
	TIM22 -> CNT &= 0xFFFF0000; // Reset counter.
	TIM22 -> SR &= 0xFFFFF9B8; // Clear all flags.
	// Configure TIM22 as slave to count seconds.
	TIM22 -> SMCR &= ~(0b111 << 4); // TS = '000' to select ITR0 = TIM1 as trigger input.
	TIM22 -> SMCR |= (0b111 << 0); // SMS = '111' to enable slave mode with external clock.
	// Generate event to update registers.
	TIM22 -> EGR |= (0b1 << 0); // UG='1'.
}

/* ENABLE TIM22 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Start(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 5); // TIM22EN='1'.
	// Enable TIM22 peripheral.
	TIM22 -> CR1 |= (0b1 << 0); // Enable TIM22 (CEN='1').
}

/* STOP TIM22 COUNTER.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Stop(void) {
	// Disable TIM22 peripheral.
	TIM22 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* DISABLE TIM22 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Disable(void) {
	// Disable TIM22 peripheral.
	TIM22 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM22 -> CNT = 0;
	// Clear all flags.
	TIM22 -> SR &= 0xFFFFF9B8;
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 5); // TIM22EN='0'.
}

/* RETURNS THE NUMBER OF SECONDS ELLAPSED SINCE START-UP.
 * @param:	None.
 * @return:	Number of seconds ellapsed since start-up.
 */
volatile unsigned int TIM22_GetSeconds(void) {
	return (TIM22 -> CNT);
}
