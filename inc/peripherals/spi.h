/*
 * spi.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef __SPI_H__
#define __SPI_H__

#include "types.h"

/*** SPI structures ***/

/*!******************************************************************
 * \enum SPI_status_t
 * \brief SPI driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	SPI_SUCCESS = 0,
	SPI_ERROR_NULL_PARAMETER,
	SPI_ERROR_TX_BUFFER_EMPTY,
	SPI_ERROR_RX_TIMEOUT,
	// Last base value.
	SPI_ERROR_BASE_LAST = 0x0100
} SPI_status_t;

/*** SPI functions ***/

/*!******************************************************************
 * \fn void SPI1_init(void)
 * \brief Init SPI1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI1_init(void);

/*!******************************************************************
 * \fn void SPI1_de_init(void)
 * \brief Release SPI1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI1_de_init(void);

/*!******************************************************************
 * \fn SPI_status_t SPI1_write_read(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size)
 * \brief SPI1 data transfer function.
 * \param[in]	tx_data: Byte array to send.
 * \param[in]	transfer_size: Number of bytes to send and receive.
 * \param[out] 	rx_data: Pointer to the received bytes.
 * \retval		Function execution status.
 *******************************************************************/
SPI_status_t SPI1_write_read(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size);

/*******************************************************************/
#define SPI1_exit_error(error_base) { if (spi1_status != SPI_SUCCESS) { status = (error_base + spi1_status); goto errors; } }

/*******************************************************************/
#define SPI1_stack_error(void) { if (spi1_status != SPI_SUCCESS) { ERROR_stack_add(ERROR_BASE_SPI1 + spi1_status); } }

/*******************************************************************/
#define SPI1_stack_exit_error(error_code) { if (spi1_status != SPI_SUCCESS) { ERROR_stack_add(ERROR_BASE_SPI1 + spi1_status); status = error_code; goto errors; } }

#endif /* __SPI_H__ */
