/*
 * usart.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "usart.h"

#include "at.h"
#include "gpio.h"
#include "lptim.h"
#include "mode.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "usart_reg.h"

#ifdef ATM
/*** USART local macros ***/

//#define USE_TXE_INTERRUPT
#define USART_BAUD_RATE			9600
#define USART_TX_BUFFER_SIZE	128
#define USART_TIMEOUT_COUNT		100000

/*** USART local structures ***/

#ifdef USE_TXE_INTERRUPT
typedef struct {
	unsigned char tx_buf[USART_TX_BUFFER_SIZE]; 	// Transmit buffer.
	unsigned int tx_buf_read_idx; 					// Reading index in TX buffer.
	unsigned int tx_buf_write_idx; 					// Writing index in TX buffer.
} USART_Context;
#endif

/*** USART local global variables ***/

#ifdef USE_TXE_INTERRUPT
static volatile USART_Context usart_ctx;
#endif

/*** USART local functions ***/

/* USART2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) USART2_IRQHandler(void) {
#ifdef USE_TXE_INTERRUPT
	// TXE interrupt.
	if (((USART2 -> ISR) & (0b1 << 7)) != 0) {
		if ((usart_ctx.tx_buf_read_idx) != (usart_ctx.tx_buf_write_idx)) {
			USART2 -> TDR = (usart_ctx.tx_buf)[usart_ctx.tx_buf_read_idx]; // Fill transmit data register with new byte.
			usart_ctx.tx_buf_read_idx++; // Increment TX read index.
			if (usart_ctx.tx_buf_read_idx == USART_TX_BUFFER_SIZE) {
				usart_ctx.tx_buf_read_idx = 0; // Manage roll-over.
			}
		}
		else {
			// No more bytes, disable TXE interrupt.
			USART2 -> CR1 &= ~(0b1 << 7); // TXEIE = '0'.
		}
	}
#endif
	// RXNE interrupt.
	if (((USART2 -> ISR) & (0b1 << 5)) != 0) {
		// Transmit incoming byte to AT command manager.
		AT_FillRxBuffer(USART2 -> RDR);
		// Clear RXNE flag.
		USART2 -> RQR |= (0b1 << 3);
	}
	// Overrun error interrupt.
	if (((USART2 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART2 -> ICR |= (0b1 << 3);
	}
}

/* FILL USART TX BUFFER WITH A NEW BYTE.
 * @param tx_byte:	Byte to append.
 * @return:			None.
 */
static void USART2_FillTxBuffer(unsigned char tx_byte) {
#ifdef USE_TXE_INTERRUPT
	// Fill buffer.
	usart_ctx.tx_buf[usart_ctx.tx_buf_write_idx] = tx_byte;
	// Manage index roll-over.
	usart_ctx.tx_buf_write_idx++;
	if (usart_ctx.tx_buf_write_idx == USART_TX_BUFFER_SIZE) {
		usart_ctx.tx_buf_write_idx = 0;
	}
#else
	// Fill transmit register.
	USART2 -> TDR = tx_byte;
	// Wait for transmission to complete.
	unsigned int loop_count = 0;
	while (((USART2 -> ISR) & (0b1 << 7)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > USART_TIMEOUT_COUNT) break;
	}
#endif
}
#endif

/*** USART functions ***/

/* CONFIGURE USART2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART2_Init(void) {
#ifdef ATM
#ifdef USE_TXE_INTERRUPT
	// Init context.
	unsigned int idx = 0;
	for (idx=0 ; idx<USART_TX_BUFFER_SIZE ; idx++) usart_ctx.tx_buf[idx] = 0;
	usart_ctx.tx_buf_write_idx = 0;
	usart_ctx.tx_buf_read_idx = 0;
#endif
	// Enable peripheral clock.
	RCC -> CR |= (0b1 << 1); // Enable HSI in stop mode (HSI16KERON='1').
	RCC -> CCIPR |= (0b10 << 2); // Select HSI as USART clock.
	RCC -> APB1ENR |= (0b1 << 17); // USART2EN='1'.
	RCC -> APB1SMENR |= (0b1 << 17); // Enable clock in sleep mode.
	// Configure TX and RX GPIOs.
	GPIO_Configure(&GPIO_USART2_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_USART2_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Configure peripheral.
	USART2 -> CR3 |= (0b1 << 12) | (0b1 << 23); // No overrun detection (OVRDIS='1') and clock enable in stop mode (UCESM='1').
	USART2 -> BRR = ((RCC_HSI_FREQUENCY_KHZ * 1000) / (USART_BAUD_RATE)); // BRR = (fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Enable transmitter and receiver.
	USART2 -> CR1 |= (0b1 << 5) | (0b11 << 2); // TE='1', RE='1' and RXNEIE='1'.
	// Set interrupt priority.
	NVIC_SetPriority(NVIC_IT_USART2, 3);
	// Enable peripheral.
	USART2 -> CR1 |= (0b11 << 0);
#else
	// Configure TX and RX GPIOs.
	GPIO_Configure(&GPIO_USART2_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_USART2_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}

#ifdef ATM
/* SEND A BYTE ARRAY THROUGH USART2.
 * @param tx_string:	Byte array to send.
 * @return:				None.
 */
void USART2_SendString(char* tx_string) {
	// Disable interrupt.
	NVIC_DisableInterrupt(NVIC_IT_USART2);
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		USART2_FillTxBuffer((unsigned char) *(tx_string++));
	}
	// Enable interrupt.
	#ifdef USE_TXE_INTERRUPT
		USART2 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
	#endif
	NVIC_EnableInterrupt(NVIC_IT_USART2);
}
#endif
