/*
 * spi.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __SPI_H__
#define __SPI_H__

#include "lptim.h"
#include "types.h"

/*** SPI structures ***/

typedef enum {
	SPI_SUCCESS = 0,
	SPI_ERROR_NULL_PARAMETER,
	SPI_ERROR_TX_BUFFER_EMPTY,
	SPI_ERROR_RX_TIMEOUT,
	SPI_ERROR_BASE_LPTIM = 0x0100,
	SPI_ERROR_BASE_LAST = (SPI_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} SPI_status_t;

/*** SPI functions ***/

void SPI1_init(void);
SPI_status_t SPI1_power_on(void);
void SPI1_power_off(void);
SPI_status_t SPI1_write_byte(uint8_t tx_data);
SPI_status_t SPI1_read_byte(uint8_t tx_data, uint8_t* rx_data);

#define SPI1_status_check(error_base) { if (spi_status != SPI_SUCCESS) { status = error_base + spi_status; goto errors; }}
#define SPI1_error_check() { ERROR_status_check(spi_status, SPI_SUCCESS, ERROR_BASE_SPI1); }
#define SPI1_error_check_print() { ERROR_status_check_print(spi_status, SPI_SUCCESS, ERROR_BASE_SPI1); }

#endif /* __SPI_H__ */
