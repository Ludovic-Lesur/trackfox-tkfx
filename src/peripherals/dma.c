/*
 * dma.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "dma.h"

#include "dma_reg.h"
#include "lpuart_reg.h"
#include "neom8n.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "spi_reg.h"

/*** DMA local functions ***/

/* DMA1 CHANNEL 6 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Channel4_5_6_7_IRQHandler(void) {
	// Transfer complete interrupt (TCIF6='1').
	if (((DMA1 -> ISR) & (0b1 << 21)) != 0) {
		// Switch DMA buffer without decoding.
		NEOM8N_SwitchDmaBuffer(0);
		// Clear flag.
		DMA1 -> IFCR |= (0b1 << 21); // CTCIF6='1'.
	}
}

/* CONFIGURE DMA1 CHANNEL3 FOR SPI1 TX TRANSFER (S2LP TX POLAR MODULATION).
 * @param:	None.
 * @return:	None.
 */
void DMA1_InitChannel3(void) {
	// Enable peripheral clock.
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='1'.
	// Disable DMA channel before configuration (EN='0').
	// Memory and peripheral data size are 8 bits (MSIZE='00' and PSIZE='00').
	// Disable memory to memory mode (MEM2MEM='0').
	// Peripheral increment mode disabled (PINC='0').
	// Circular mode disabled (CIRC='0').
	// Read from memory (DIR='1').
	DMA1 -> CCR3 |= (0b11 << 12); // Very high priority (PL='11').
	DMA1 -> CCR3 |= (0b1 << 7); // Memory increment mode enabled (MINC='1').
	DMA1 -> CCR3 &= ~(0b1 << 1); // Disable transfer complete interrupt (TCIE='1').
	DMA1 -> CCR3 |= (0b1 << 4); // Read from memory.
	// Configure peripheral address.
	DMA1 -> CPAR3 = (unsigned int) &(SPI1 -> DR); // Peripheral address = SPI1 TX register.
	// Configure channel 3 for SPI1 TX (request number 1).
	DMA1 -> CSELR &= ~(0b1111 << 8); // Reset bits 8-11.
	DMA1 -> CSELR |= (0b0001 << 8); // DMA channel mapped on SPI1_TX (C3S='0001').
	// Clear all flags.
	DMA1 -> IFCR |= 0x00000F00;
}

/* START DMA1 CHANNEL 3 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_StartChannel3(void) {
	// Clear all flags.
	DMA1 -> IFCR |= 0x00000F00;
	// Start transfer.
	DMA1 -> CCR3 |= (0b1 << 0); // EN='1'.
}

/* STOP DMA1 CHANNEL 3 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_StopChannel3(void) {
	// Stop transfer.
	DMA1 -> CCR3 &= ~(0b1 << 0); // EN='0'.
}

/* SET DMA1 CHANNEL 3 SOURCE BUFFER ADDRESS.
 * @param dest_buf_addr:	Address of source buffer (Sigfox modulation stream).
 * @param dest_buf_size:	Size of destination buffer.
 * @return:					None.
 */
void DMA1_SetChannel3SourceAddr(unsigned int source_buf_addr, unsigned short source_buf_size) {
	// Set address.
	DMA1 -> CMAR3 = source_buf_addr;
	// Set buffer size.
	DMA1 -> CNDTR3 = source_buf_size;
	// Clear all flags.
	DMA1 -> IFCR |= 0x00000F00;
}

/* GET DMA1 CHANNEL 3 TRANSFER STATUS.
 * @param:	None.
 * @return:	'1' if the transfer is complete, '0' otherwise.
 */
unsigned char DMA1_GetChannel3Status(void) {
	return ((unsigned char) (((DMA1 -> ISR) >> 9) & 0x01)); // Read TCIF3.
}

/* CONFIGURE DMA1 CHANNEL 6 FOR LPUART RX TRANSFER (NMEA FRAMES FROM GPS MODULE).
 * @param:	None.
 * @return:	None.
 */
void DMA1_InitChannel6(void) {
	// Enable peripheral clock.
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='1'.
	// Disable DMA channel before configuration (EN='0').
	// Memory and peripheral data size are 8 bits (MSIZE='00' and PSIZE='00').
	// Disable memory to memory mode (MEM2MEM='0').
	// Peripheral increment mode disabled (PINC='0').
	// Circular mode disabled (CIRC='0').
	// Read from peripheral (DIR='0').
	DMA1 -> CCR6 |= (0b11 << 12); // Very high priority (PL='11').
	DMA1 -> CCR6 |= (0b1 << 7); // Memory increment mode enabled (MINC='1').
	DMA1 -> CCR6 |= (0b1 << 1); // Enable transfer complete interrupt (TCIE='1').
	DMA1 -> CCR6 &= ~(0b1 << 4); // Read from peripheral.
	// Configure peripheral address.
	DMA1 -> CPAR6 = (unsigned int) &(LPUART1 -> RDR); // Peripheral address = LPUART RX register.
	// Configure channel 3 for LPUART1 RX (request number 5).
	DMA1 -> CSELR &= ~(0b1111 << 20); // Reset bits 20-23.
	DMA1 -> CSELR |= (0b0101 << 20); // DMA channel mapped on LPUART1_RX (C6S='0101').
	// Clear all flags.
	DMA1 -> IFCR |= 0x00F00000;
}

/* START DMA1 CHANNEL 6 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_StartChannel6(void) {
	// Clear all flags.
	DMA1 -> IFCR |= 0x00F00000;
	NVIC_EnableInterrupt(IT_DMA1_Channel4_7);
	// Start transfer.
	DMA1 -> CCR6 |= (0b1 << 0); // EN='1'.
}

/* STOP DMA1 CHANNEL 6 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_StopChannel6(void) {
	// Stop transfer.
	DMA1 -> CCR6 &= ~(0b1 << 0); // EN='0'.
	NVIC_DisableInterrupt(IT_DMA1_Channel4_7);
}

/* SET DMA1 CHANNEL 6 DESTINATION BUFFER ADDRESS.
 * @param dest_buf_addr:	Address of destination buffer (NMEA frame).
 * @param dest_buf_size:	Size of destination buffer.
 * @return:					None.
 */
void DMA1_SetChannel6DestAddr(unsigned int dest_buf_addr, unsigned short dest_buf_size) {
	// Set address.
	DMA1 -> CMAR6 = dest_buf_addr;
	// Set buffer size.
	DMA1 -> CNDTR6 = dest_buf_size;
	// Clear all flags.
	DMA1 -> IFCR |= 0x00F00000;
}

/* DISABLE DMA1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Disable(void) {
	// Disable interrupts.
	NVIC_DisableInterrupt(IT_DMA1_Channel4_7);
	// Clear all flags.
	DMA1 -> IFCR |= 0x0FFFFFFF;
	// Disable peripheral clock.
	RCC -> AHBENR &= ~(0b1 << 0); // DMAEN='0'.
}
