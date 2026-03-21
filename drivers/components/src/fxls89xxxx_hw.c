/*
 * fxls89xxxx_hw.c
 *
 *  Created on: 17 oct. 2025
 *      Author: Ludo
 */

#include "fxls89xxxx_hw.h"

#ifndef FXLS89XXXX_DRIVER_DISABLE_FLAGS_FILE
#include "fxls89xxxx_driver_flags.h"
#endif
#include "error_base.h"
#include "fxls89xxxx.h"
#include "sensors_hw.h"
#include "types.h"

#ifndef FXLS89XXXX_DRIVER_DISABLE

/*** FXLS89XXXX HW functions ***/

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_init(void) {
    return ((FXLS89XXXX_status_t) SENSORS_HW_init(FXLS89XXXX_ERROR_BASE_I2C));
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_de_init(void) {
    return ((FXLS89XXXX_status_t) SENSORS_HW_de_init(ERROR_BASE_FXLS8974CF + FXLS89XXXX_ERROR_BASE_I2C));
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((FXLS89XXXX_status_t) SENSORS_HW_i2c_write(FXLS89XXXX_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((FXLS89XXXX_status_t) SENSORS_HW_i2c_read(FXLS89XXXX_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

#endif /* FXLS89XXXX_DRIVER_DISABLE */
