/*
 * spi.c
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#include "spi.h"

#include "gpio.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "spi_reg.h"
#include "types.h"

/*** SPI local macros ***/

#define SPI_ACCESS_TIMEOUT_COUNT	1000000

/*** SPI functions ***/

/*******************************************************************/
void SPI1_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.
	// Master mode (MSTR='1').
	// Baud rate = PCLK2/2 = SYSCLK/2 = 8MHz.
	// 8-bits format (DFF='0').
	// Enable output (SSOE='1').
	SPI1 -> CR1 |= (0b1 << 2);
	SPI1 -> CR2 |= (0b1 << 2);
	// Enable peripheral.
	SPI1 -> CR1 |= (0b1 << 6); // SPE='1'.
	// Configure GPIOs.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
}

/*******************************************************************/
void SPI1_de_init(void) {
	// Disable SPI alternate function.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable peripheral.
	SPI1 -> CR1 &= ~(0b1 << 6); // SPE='0'.
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 12); // SPI1EN='0'.
}

/*******************************************************************/
SPI_status_t SPI1_write_read(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint8_t transfer_idx = 0;
	uint32_t loop_count = 0;
	// Check parameters.
	if ((tx_data == NULL) || (rx_data == NULL)) {
		status = SPI_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Transfer loop.
	for (transfer_idx=0 ; transfer_idx<transfer_size ; transfer_idx++) {
		// Dummy read to DR to clear RXNE flag.
		rx_data[transfer_idx] = *((volatile uint8_t*) &(SPI1 -> DR));
		// Wait for TXE flag.
		while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
			// Wait for TXE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = SPI_ERROR_TX_BUFFER_EMPTY;
				goto errors;
			}
		}
		// Send TX byte.
		*((volatile uint8_t*) &(SPI1 -> DR)) = tx_data[transfer_idx];
		// Wait for incoming data.
		loop_count = 0;
		while (((SPI1 -> SR) & (0b1 << 0)) == 0) {
			// Wait for RXNE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = SPI_ERROR_RX_TIMEOUT;
				goto errors;
			}
		}
		rx_data[transfer_idx] = *((volatile uint8_t*) &(SPI1 -> DR));
	}
errors:
	return status;
}
