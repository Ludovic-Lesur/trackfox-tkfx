/*
 * lpuart.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __LPUART_H__
#define __LPUART_H__

#include "types.h"

/*** LPUART structures ***/

/*!******************************************************************
 * \enum LPUART_status_t
 * \brief LPUART driver error codes.
 *******************************************************************/
typedef enum {
	LPUART_SUCCESS = 0,
	LPUART_ERROR_NULL_PARAMETER,
	LPUART_ERROR_TX_TIMEOUT,
	LPUART_ERROR_BASE_LAST = 0x0100
} LPUART_status_t;

/*!******************************************************************
 * \fn LPUART_character_match_irq_cb_t
 * \brief LPUART character match interrupt callback.
 *******************************************************************/
typedef void (*LPUART_character_match_irq_cb_t)(void);

/*** LPUART functions ***/

/*!******************************************************************
 * \fn void void LPUART1_init(LPUART_character_match_irq_cb_t irq_callback)
 * \brief Init LPUART1 peripheral.
 * \param[in]  	irq_callback: Function to call on character match interrupt.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LPUART1_init(LPUART_character_match_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn void LPUART1_de_init(void)
 * \brief Release LPUART1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LPUART1_de_init(void);

/*!******************************************************************
 * \fn LPUART_status_t LPUART1_write(uint8_t* data, uint8_t data_size_bytes)
 * \brief Send data over LPUART1.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPUART_status_t LPUART1_write(uint8_t* data, uint32_t data_size_bytes);

/*******************************************************************/
#define LPUART1_check_status(error_base) { if (lpuart1_status != LPUART_SUCCESS) { status = error_base + lpuart1_status; goto errors; } }

/*******************************************************************/
#define LPUART1_stack_error(void) { ERROR_stack_error(lpuart1_status, LPUART_SUCCESS, ERROR_BASE_LPUART1); }

/*******************************************************************/
#define LPUART1_print_error(void) { ERROR_print_error(lpuart1_status, LPUART_SUCCESS, ERROR_BASE_LPUART1); }

#endif /* __LPUART_H__ */
