/*
 * maxm10x_hw.c
 *
 *  Created on: 15 mar. 2026
 *      Author: Ludo
 */

#include "maxm10x_hw.h"

#ifndef MAXM10X_DRIVER_DISABLE_FLAGS_FILE
#include "maxm10x_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "lptim.h"
#include "maxm10x.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "types.h"
#include "usart.h"

#ifndef MAXM10X_DRIVER_DISABLE

/*** MAXM10X HW functions ***/

/*******************************************************************/
MAXM10X_status_t MAXM10X_HW_init(MAXM10X_HW_configuration_t* configuration) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    USART_configuration_t usart_config;
    // Init backup pin.
    GPIO_configure(&GPIO_GPS_VBCKP, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Init USART.
    usart_config.clock = RCC_CLOCK_SYSTEM;
    usart_config.baud_rate = (configuration->uart_baud_rate);
    usart_config.nvic_priority = NVIC_PRIORITY_GPS_UART;
    usart_config.rxne_irq_callback = (USART_rx_irq_cb_t) (configuration->rx_irq_callback);
    usart_status = USART_init(USART_INSTANCE_GPS, &USART_GPIO_GPS, &usart_config);
    USART_exit_error(MAXM10X_ERROR_BASE_UART);
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_HW_de_init(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Release USART.
    usart_status = USART_de_init(USART_INSTANCE_GPS, &USART_GPIO_GPS);
    USART_stack_error(ERROR_BASE_MAXM10S + MAXM10X_ERROR_BASE_UART);
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_HW_send_message(uint8_t* message, uint32_t message_size_bytes) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Use USART.
    usart_status = USART_write(USART_INSTANCE_GPS, message, message_size_bytes);
    USART_exit_error(MAXM10X_ERROR_BASE_UART);
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_HW_start_rx(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Start USART.
    usart_status = USART_enable_rx(USART_INSTANCE_GPS);
    USART_exit_error(MAXM10X_ERROR_BASE_UART);
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_HW_stop_rx(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Stop USART.
    usart_status = USART_disable_rx(USART_INSTANCE_GPS);
    USART_exit_error(MAXM10X_ERROR_BASE_UART);
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(MAXM10X_ERROR_BASE_DELAY);
errors:
    return status;
}

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
MAXM10X_status_t MAXM10X_HW_set_backup_voltage(uint8_t state) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Set GPIO.
    GPIO_write(&GPIO_GPS_VBCKP, state);
    return status;
}
#endif

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
MAXM10X_status_t  MAXM10X_HW_get_backup_voltage(uint8_t* state) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Read GPIO.
    (*state) = GPIO_read(&GPIO_GPS_VBCKP);
    return status;
}
#endif

#endif /* MAXM10X_DRIVER_DISABLE */
