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
	GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
	// Configure NSS, SCK, MISO and MOSI (first as high impedance).
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_configure(&GPIO_S2LP_CS, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	// Configure peripheral.
	SPI1 -> CR1 &= 0xFFFF0000; // Disable peripheral before configuration (SPE='0').
	SPI1 -> CR1 |= (0b1 << 2); // Master mode (MSTR='1').
	SPI1 -> CR1 &= ~(0b111 << 3); // Baud rate = PCLK2/2 = SYSCLK/2 = 8MHz.
	SPI1 -> CR1 &= ~(0b1 << 11); // 8-bits format (DFF='0').
	SPI1 -> CR1 &= ~(0b11 << 0); // CPOL='0' and CPHA='0'.
	SPI1 -> CR2 &= 0xFFFFFF08;
	SPI1 -> CR2 |= (0b1 << 2); // Enable output (SSOE='1').
	SPI1 -> CR2 |= (0b1 << 1); // Enable TX DMA requests.
	// Enable peripheral.
	SPI1 -> CR1 |= (0b1 << 6); // SPE='1'.
}

/* ENABLE SPI1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void SPI1_enable(void) {
	// Enable SPI1 peripheral.
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.
	SPI1 -> CR1 |= (0b1 << 6);
	// Configure power enable pins.
	GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
}

/* DISABLE SPI1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void SPI1_disable(void) {
	// Disable power control pin.
	GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable SPI1 peripheral.
	SPI1 -> CR1 &= ~(0b1 << 6);
	// Clear all flags.
	SPI1 -> SR &= 0xFFFFFEEF;
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 12); // SPI1EN='0'.
}

/* SWITCH ALL SPI1 SLAVES ON.
 * @param:	None.
 * @return:	None.
 */
void SPI1_power_on(void) {
	// Turn S2LP on.
	GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
	// Wait for power-on.
	LPTIM1_delay_milliseconds(50, 1);
	// Enable GPIOs.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_write(&GPIO_S2LP_CS, 1); // CS high (idle state).
	GPIO_configure(&GPIO_S2LP_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Wait for power-on.
	LPTIM1_delay_milliseconds(50, 1);
}

/* SWITCH ALL SPI1 SLAVES OFF.
 * @param:	None.
 * @return:	None.
 */
void SPI1_power_off(void) {
	// Turn S2LP off.
	GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
	GPIO_write(&GPIO_S2LP_CS, 0); // CS low (to avoid powering slaves via SPI bus).
	// Disable SPI alternate function.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_configure(&GPIO_S2LP_CS, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	// Wait for power-off.
	LPTIM1_delay_milliseconds(100, 1);
}

/* SEND A BYTE THROUGH SPI1.
 * @param tx_data:	Data to send (8-bits).
 * @return:			1 in case of success, 0 in case of failure.
 */
unsigned char SPI1_write_byte(unsigned char tx_data) {
	// Wait for TXE flag.
	unsigned int loop_count = 0;
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) return 0;
	}
	// Send data.
	*((volatile unsigned char*) &(SPI1 -> DR)) = tx_data;
	return 1;
}

/* READ A BYTE FROM SPI1.
 * @param rx_data:	Pointer to byte that will contain the data to read (8-bits).
 * @return:			1 in case of success, 0 in case of failure.
 */
unsigned char SPI1_read_byte(unsigned char tx_data, unsigned char* rx_data) {
	// Dummy read to DR to clear RXNE flag.
	(*rx_data) = *((volatile unsigned char*) &(SPI1 -> DR));
	// Wait for TXE flag.
	unsigned int loop_count = 0;
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) return 0;
	}
	// Send dummy data on MOSI to generate clock.
	*((volatile unsigned char*) &(SPI1 -> DR)) = tx_data;
	// Wait for incoming data.
	loop_count = 0;
	while (((SPI1 -> SR) & (0b1 << 0)) == 0) {
		// Wait for RXNE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) return 0;
	}
	(*rx_data) = *((volatile unsigned char*) &(SPI1 -> DR));
	return 1;
}
