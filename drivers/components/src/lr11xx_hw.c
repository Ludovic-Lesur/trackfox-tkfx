/*
 * lr11xx_hw.c
 *
 *  Created on: 07 mar. 2026
 *      Author: Ludo
 */

#include "lr11xx_hw.h"

#ifndef LR11XX_DRIVER_DISABLE_FLAGS_FILE
#include "lr11xx_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "lptim.h"
#include "lr11xx.h"
#include "mcu_mapping.h"
#include "spi.h"
#include "types.h"

#ifndef LR11XX_DRIVER_DISABLE

/*** SX126X HW local macros ***/

#define LR11XX_HW_BUSY_TIMEOUT_COUNT    1000000

/*** LR11XX HW functions ***/

/*******************************************************************/
LR11XX_status_t LR11XX_HW_init(void) {
    // Local variables.
    LR11XX_status_t status = LR11XX_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    SPI_configuration_t spi_config;
    // Configure reset pin.
    GPIO_configure(&GPIO_LR1110_NRESET, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_LR1110_NRESET, 0);
    // Configure busy pin.
    GPIO_configure(&GPIO_LR1110_BUSY, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    // Init SPI.
    spi_config.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_4;
    spi_config.data_format = SPI_DATA_FORMAT_8_BITS;
    spi_config.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_status = SPI_init(SPI_INSTANCE_RADIO, &SPI_GPIO_LR1110, &spi_config);
    SPI_exit_error(LR11XX_ERROR_BASE_SPI);
    // Configure chip select pin.
    GPIO_configure(&GPIO_LR1110_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_LR1110_CS, 1);
    errors:
    return status;
}

/*******************************************************************/
LR11XX_status_t LR11XX_HW_de_init(void) {
    // Local variables.
    LR11XX_status_t status = LR11XX_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // Keep reset pin as output low.
    GPIO_write(&GPIO_LR1110_NRESET, 0);
    // Switch busy pin to output low.
    GPIO_configure(&GPIO_LR1110_BUSY, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_LR1110_BUSY, 0);
    // Keep chip select pin as output low.
    GPIO_write(&GPIO_LR1110_CS, 0);
    // Release SPI.
    spi_status = SPI_de_init(SPI_INSTANCE_RADIO, &SPI_GPIO_LR1110);
    SPI_stack_error(ERROR_BASE_LR1110 + LR11XX_ERROR_BASE_SPI);
    return status;
}

/*******************************************************************/
LR11XX_status_t LR11XX_HW_set_nreset_gpio(uint8_t state) {
    // Local variables.
    LR11XX_status_t status = LR11XX_SUCCESS;
    // Set GPIO state.
    GPIO_write(&GPIO_LR1110_NRESET, state);
    return status;
}

/*******************************************************************/
LR11XX_status_t LR11XX_HW_wait_busy_low(void) {
    // Local variables.
    LR11XX_status_t status = LR11XX_SUCCESS;
    uint32_t loop_count = 0;
    // Wait for busy pin to be low.
    while (GPIO_read(&GPIO_LR1110_BUSY) != 0) {
        // Exit of timeout.
        loop_count++;
        if (loop_count > LR11XX_HW_BUSY_TIMEOUT_COUNT) {
            status = LR11XX_ERROR_BUSY_TIMEOUT;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
LR11XX_status_t LR11XX_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    LR11XX_status_t status = LR11XX_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // CS low.
    GPIO_write(&GPIO_LR1110_CS, 0);
    // SPI transfer.
    spi_status = SPI_write_read_8(SPI_INSTANCE_RADIO, tx_data, rx_data, transfer_size);
    SPI_exit_error(LR11XX_ERROR_BASE_SPI);
errors:
    // CS high.
    GPIO_write(&GPIO_LR1110_CS, 1);
    return status;
}

/*******************************************************************/
LR11XX_status_t LR11XX_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    LR11XX_status_t status = LR11XX_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(LR11XX_ERROR_BASE_DELAY);
errors:
    return status;
}

#endif /* LR11XX_DRIVER_DISABLE */
