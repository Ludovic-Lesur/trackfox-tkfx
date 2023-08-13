/*
 * usart.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __USART_H__
#define __USART_H__

#include "mode.h"
#include "types.h"

/*** USART structures ***/

/*!******************************************************************
 * \enum USART_status_t
 * \brief USART driver error codes.
 *******************************************************************/
typedef enum {
	USART_SUCCESS = 0,
	USART_ERROR_NULL_PARAMETER,
	USART_ERROR_TX_TIMEOUT,
	USART_ERROR_BASE_LAST = 0x0100
} USART_status_t;

/*!******************************************************************
 * \fn USART_rx_irq_cb_t
 * \brief USART RX interrupt callback.
 *******************************************************************/
typedef void (*USART_rx_irq_cb_t)(uint8_t data);

/*** USART functions ***/

#ifdef ATM
/*!******************************************************************
 * \fn void USART2_init(USART_rx_irq_cb_t irq_callback)
 * \brief Init USART2 peripheral.
 * \param[in]  	irq_callback: Function to call on interrupt.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_init(USART_rx_irq_cb_t irq_callback);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void USART2_de_init(void)
 * \brief Release USART2 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_de_init(void);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void USART2_enable_rx(void)
 * \brief Enable USART2 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_enable_rx(void);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void USART2_disable_rx(void)
 * \brief Disable USART2 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_disable_rx(void);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn USART_status_t USART2_write(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over USART2.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART2_write(uint8_t* data, uint32_t data_size_bytes);
#endif

/*******************************************************************/
#define USART2_check_status(error_base) { if (usart2_status != USART_SUCCESS) { status = error_base + usart2_status; goto errors; } }

/*******************************************************************/
#define USART2_stack_error(void) { ERROR_stack_error(usart2_status, USART_SUCCESS, ERROR_BASE_USART2); }

/*******************************************************************/
#define USART2_print_error(void) { ERROR_print_error(usart2_status, USART_SUCCESS, ERROR_BASE_USART2); }

#endif /* __USART_H__ */
