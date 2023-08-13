/*
 * error.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_H__
#define __ERROR_H__

// Peripherals.
#include "adc.h"
#include "aes.h"
#include "flash.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "nvm.h"
#include "rcc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
// Components.
#include "mma8653fc.h"
#include "neom8n.h"
#include "power.h"
#include "s2lp.h"
#include "sht3x.h"
// Utils.
#include "math.h"
#include "parser.h"
#include "string.h"

/*!******************************************************************
 * \enum ERROR_t
 * \brief Board error codes.
 *******************************************************************/
typedef enum {
	SUCCESS = 0,
	ERROR_BUSY,
	ERROR_SIGFOX_RC,
	// Peripherals.
	ERROR_BASE_ADC1 = 0x0100,
	ERROR_BASE_AES = (ERROR_BASE_ADC1 + ADC_ERROR_BASE_LAST),
	ERROR_BASE_FLASH = (ERROR_BASE_AES + AES_ERROR_BASE_LAST),
	ERROR_BASE_I2C1 = (ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
	ERROR_BASE_IWDG = (ERROR_BASE_I2C1 + I2C_ERROR_BASE_LAST),
	ERROR_BASE_LPTIM1 = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
	ERROR_BASE_LPUART1 = (ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST),
	ERROR_BASE_NVM = (ERROR_BASE_LPUART1 + LPUART_ERROR_BASE_LAST),
	ERROR_BASE_RCC = (ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
	ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
	ERROR_BASE_SPI1 = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	ERROR_BASE_TIM2 = (ERROR_BASE_SPI1 + SPI_ERROR_BASE_LAST),
	ERROR_BASE_TIM21 = (ERROR_BASE_TIM2 + TIM_ERROR_BASE_LAST),
	ERROR_BASE_USART2 = (ERROR_BASE_TIM21 + TIM_ERROR_BASE_LAST),
	// Utils.
	ERROR_BASE_MATH = (ERROR_BASE_USART2 + USART_ERROR_BASE_LAST),
	ERROR_BASE_PARSER = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
	ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
	// Components.
	ERROR_BASE_MMA8653FC = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	ERROR_BASE_NEOM8N = (ERROR_BASE_MMA8653FC + MMA8653FC_ERROR_BASE_LAST),
	ERROR_BASE_POWER = (ERROR_BASE_NEOM8N + NEOM8N_ERROR_BASE_LAST),
	ERROR_BASE_S2LP = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
	ERROR_BASE_SHT3X = (ERROR_BASE_NEOM8N + NEOM8N_ERROR_BASE_LAST),
	// Libraries.
	ERROR_BASE_SIGFOX_MCU_API = (ERROR_BASE_SHT3X + SHT3X_ERROR_BASE_LAST),
	ERROR_BASE_SIGFOX_RF_API = (ERROR_BASE_SIGFOX_MCU_API + 0x0100),
	ERROR_BASE_SIGFOX_EP_API = (ERROR_BASE_SIGFOX_RF_API + 0x0100),
	ERROR_BASE_SIGFOX_EP_ADDON_RFP = (ERROR_BASE_SIGFOX_EP_API + 0x0100),
	ERROR_BASE_LAST = (ERROR_BASE_SIGFOX_EP_ADDON_RFP + 0x0100)
} ERROR_t;

/*** ERROR functions ***/

/*!******************************************************************
 * \fn void ERROR_stack_init(void)
 * \brief Init error stack.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void ERROR_stack_init(void);

/*!******************************************************************
 * \fn void ERROR_stack_add(ERROR_t code)
 * \brief Add error to stack.
 * \param[in]  	code: Error to stack.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void ERROR_stack_add(ERROR_t code);

/*!******************************************************************
 * \fn ERROR_t ERROR_stack_read(void)
 * \brief Read error stack.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Last error code stored.
 *******************************************************************/
ERROR_t ERROR_stack_read(void);

/*!******************************************************************
 * \fn uint8_t ERROR_stack_is_empty(void)
 * \brief Check if error stack is empty.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		1 if the error stack is empty, 0 otherwise.
 *******************************************************************/
uint8_t ERROR_stack_is_empty(void);

/*******************************************************************/
#define ERROR_stack_error(status, success, error_base) { \
	if (status != success) { \
		ERROR_stack_add(error_base + status); \
	} \
}

/*******************************************************************/
#define ERROR_print_error(status, success, error_base) { \
	if (status != success) { \
		_AT_print_error(error_base + status); \
		goto errors; \
	} \
}

#endif /* __ERROR_H__ */
