/*
 * mma865xfc_hw.c
 *
 *  Created on: 27 aug. 2024
 *      Author: Ludo
 */

#include "mma865xfc_hw.h"

#ifndef MMA865XFC_DRIVER_DISABLE_FLAGS_FILE
#include "mma865xfc_driver_flags.h"
#endif
#include "mma865xfc.h"
#include "sensors_hw.h"
#include "types.h"

#ifndef MMA865XFC_DRIVER_DISABLE

/*** MMA865XFC HW functions ***/

/*******************************************************************/
MMA865XFC_status_t MMA865XFC_HW_init(void) {
    return ((MMA865XFC_status_t) SENSORS_HW_init(MMA865XFC_ERROR_BASE_I2C));
}

/*******************************************************************/
MMA865XFC_status_t MMA865XFC_HW_de_init(void) {
    return ((MMA865XFC_status_t) SENSORS_HW_de_init(MMA865XFC_ERROR_BASE_I2C));
}

/*******************************************************************/
MMA865XFC_status_t MMA865XFC_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((MMA865XFC_status_t) SENSORS_HW_i2c_write(MMA865XFC_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
MMA865XFC_status_t MMA865XFC_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((MMA865XFC_status_t) SENSORS_HW_i2c_read(MMA865XFC_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

#endif /* MMA865XFC_DRIVER_DISABLE */
