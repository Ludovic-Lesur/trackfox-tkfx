/*
 * sensors_hw.c
 *
 *  Created on: 16 nov. 2024
 *      Author: Ludo
 */

#include "sensors_hw.h"

#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "i2c.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "types.h"

/*** SENSORS HW local global variables ***/

static EXTI_gpio_irq_cb_t sensors_hw_accelerometer_irq_callback = NULL;

/*** SENSORS HW functions ***/

/*******************************************************************/
ERROR_code_t SENSORS_HW_init(ERROR_code_t i2c_error_base) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // Init I2C.
    i2c_status = I2C_init(I2C_INSTANCE_SENSORS, &I2C_GPIO_SENSORS);
    I2C_exit_error(i2c_error_base);
    // Configure accelerometer interrupt pin.
    EXTI_configure_gpio(&GPIO_ACCELERO_IRQ, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, sensors_hw_accelerometer_irq_callback, NVIC_PRIORITY_ACCELEROMETER);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_de_init(ERROR_code_t i2c_error_base) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // Init I2C.
    i2c_status = I2C_de_init(I2C_INSTANCE_SENSORS, &I2C_GPIO_SENSORS);
    I2C_exit_error(i2c_error_base);
    // Release accelerometer interrupt pin.
    EXTI_release_gpio(&GPIO_ACCELERO_IRQ, GPIO_MODE_INPUT);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_i2c_write(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // I2C transfer.
    i2c_status = I2C_write(I2C_INSTANCE_SENSORS, i2c_address, data, data_size_bytes, stop_flag);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_i2c_read(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // I2C transfer.
    i2c_status = I2C_read(I2C_INSTANCE_SENSORS, i2c_address, data, data_size_bytes);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_delay_milliseconds(ERROR_code_t delay_error_base, uint32_t delay_ms) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(delay_error_base);
errors:
    return status;
}

/*******************************************************************/
void SENSORS_HW_set_accelerometer_irq_callback(EXTI_gpio_irq_cb_t accelerometer_irq_callback) {
    // Update local pointer.
    sensors_hw_accelerometer_irq_callback = accelerometer_irq_callback;
}

/*******************************************************************/
void SENSORS_HW_enable_accelerometer_interrupt(void) {
    // Enable interrupt.
    EXTI_enable_gpio_interrupt(&GPIO_ACCELERO_IRQ);
}

/*******************************************************************/
void SENSORS_HW_disable_accelerometer_interrupt(void) {
    // Disable interrupt.
    EXTI_disable_gpio_interrupt(&GPIO_ACCELERO_IRQ);
}
