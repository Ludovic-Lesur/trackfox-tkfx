/*
 * lr11xx_driver_flags.h
 *
 *  Created on: 09 mar. 2026
 *      Author: Ludo
 */

#ifndef __LR11XX_DRIVER_FLAGS_H__
#define __LR11XX_DRIVER_FLAGS_H__

#include "lptim.h"
#include "spi.h"

/*** LR11XX driver compilation flags ***/

#ifndef HW2_0
#define LR11XX_DRIVER_DISABLE
#endif

#define LR11XX_DRIVER_GPIO_ERROR_BASE_LAST      0
#define LR11XX_DRIVER_SPI_ERROR_BASE_LAST       SPI_ERROR_BASE_LAST
#define LR11XX_DRIVER_DELAY_ERROR_BASE_LAST     LPTIM_ERROR_BASE_LAST

#define LR11XX_DRIVER_TX_ENABLE
#define LR11XX_DRIVER_RX_ENABLE
#define LR11XX_DRIVER_WIFI_ENABLE

#endif /* __LR11XX_DRIVER_FLAGS_H__ */
