/*
 * spi.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "spi.h"

#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "spi_reg.h"
#include "types.h"

/*** SPI local macros ***/

#define SPI_ACCESS_TIMEOUT_COUNT	1000000

/*** SPI functions ***/

/* CONFIGURE SPI1.
 * @param:	None.
 * @return:	None.
 */
void SPI1_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.
	// Configure power enable pins.
	GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_S2LP_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	SPI1_power_off();
	// Configure peripheral.
	// Master mode (MSTR='1').
	// Baud rate = PCLK2/2 = SYSCLK/2 = 8MHz.
	// 8-bits format (DFF='0').
	// Enable output (SSOE='1').
	// Enable TX DMA requests.
	SPI1 -> CR1 |= (0b1 << 2);
	SPI1 -> CR2 |= (0b1 << 1) | (0b1 << 2);
	// Enable peripheral.
	SPI1 -> CR1 |= (0b1 << 6); // SPE='1'.
}

/* SWITCH ALL SPI1 SLAVES ON.
 * @param:			None.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_power_on(void) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Enable GPIOs.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Turn SPI1 slaves on.
	GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
	// Wait for power-on.
	lptim1_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(SPI_ERROR_BASE_LPTIM);
	// Chip select high by default.
	GPIO_write(&GPIO_S2LP_CS, 1);
errors:
	return status;
}

/* SWITCH ALL SPI1 SLAVES OFF.
 * @param:	None.
 * @return:	None.
 */
void SPI1_power_off(void) {
	// Turn SPI1 slaves off.
	GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
	GPIO_write(&GPIO_S2LP_CS, 0);
	// Disable SPI alternate function.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* SEND A BYTE THROUGH SPI1.
 * @param tx_data:	8-bits data to send.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_write_byte(uint8_t tx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
	// Wait for TXE flag.
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send data.
	*((volatile uint8_t*) &(SPI1 -> DR)) = tx_data;
errors:
	return status;
}

/* READ A BYTE FROM SPI1.
 * @param rx_data:	Pointer to 8-bits value that will contain the data to read.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_read_byte(uint8_t tx_data, uint8_t* rx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if (rx_data == NULL) {
		status = SPI_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Dummy read to DR to clear RXNE flag.
	(*rx_data) = *((volatile uint8_t*) &(SPI1 -> DR));
	// Wait for TXE flag.
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send dummy data on MOSI to generate clock.
	*((volatile uint8_t*) &(SPI1 -> DR)) = tx_data;
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
	(*rx_data) = *((volatile uint8_t*) &(SPI1 -> DR));
errors:
	return status;
}
