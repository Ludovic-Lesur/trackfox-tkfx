/*
 * error.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_BASE_H__
#define __ERROR_BASE_H__

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
// Utils.
#include "math.h"
#include "parser.h"
#include "string.h"
// Components.
#include "mma865xfc.h"
#include "neom8x.h"
#include "s2lp.h"
#include "sht3x.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "gps.h"
#include "power.h"
// Sigfox.
#include "sigfox_error.h"

/*** ERROR BASE structures ***/

/*!******************************************************************
 * \enum ERROR_base_t
 * \brief Board error bases.
 *******************************************************************/
typedef enum {
    SUCCESS = 0,
    // Peripherals.
    ERROR_BASE_ADC = 0x0100,
    ERROR_BASE_AES = (ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
    ERROR_BASE_FLASH = (ERROR_BASE_AES + AES_ERROR_BASE_LAST),
    ERROR_BASE_I2C_SENSORS = (ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
    ERROR_BASE_IWDG = (ERROR_BASE_I2C_SENSORS + I2C_ERROR_BASE_LAST),
    ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
    ERROR_BASE_LPUART = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    ERROR_BASE_NVM = (ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
    ERROR_BASE_RCC = (ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
    ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    ERROR_BASE_SPI_RADIO = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
    ERROR_BASE_TIM_MCU_API = (ERROR_BASE_SPI_RADIO + SPI_ERROR_BASE_LAST),
    ERROR_BASE_TIM_CALIBRATION = (ERROR_BASE_TIM_MCU_API + TIM_ERROR_BASE_LAST),
    ERROR_BASE_USART_AT = (ERROR_BASE_TIM_CALIBRATION + TIM_ERROR_BASE_LAST),
    // Utils.
    ERROR_BASE_AT = (ERROR_BASE_USART_AT + USART_ERROR_BASE_LAST),
    ERROR_BASE_MATH = (ERROR_BASE_AT + AT_ERROR_BASE_LAST),
    ERROR_BASE_PARSER = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
    ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
    ERROR_BASE_TERMINAL = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
    // Components.
    ERROR_BASE_MMA8653FC = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
    ERROR_BASE_NEOM8N = (ERROR_BASE_MMA8653FC + MMA865XFC_ERROR_BASE_LAST),
    ERROR_BASE_S2LP = (ERROR_BASE_NEOM8N + NEOM8X_ERROR_BASE_LAST),
    ERROR_BASE_SHT30 = (ERROR_BASE_S2LP + S2LP_ERROR_BASE_LAST),
    // Middleware.
    ERROR_BASE_ANALOG = (ERROR_BASE_SHT30 + SHT3X_ERROR_BASE_LAST),
    ERROR_BASE_CLI = (ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_LAST),
    ERROR_BASE_GPS = (ERROR_BASE_CLI + CLI_ERROR_BASE_LAST),
    ERROR_BASE_POWER = (ERROR_BASE_GPS + GPS_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_LIB = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_ADDON_RFP = (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_LAST * ERROR_BASE_STEP)),
    // Last base value.
    ERROR_BASE_LAST = (ERROR_BASE_SIGFOX_EP_ADDON_RFP + ERROR_BASE_STEP)
} ERROR_base_t;

#endif /* __ERROR_BASE_H__ */
