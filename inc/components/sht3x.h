/*
 * sht3x.h
 *
 *  Created on: 16 feb. 2023
 *      Author: Ludo
 */

#ifndef __SHT3X_H__
#define __SHT3X_H__

#include "i2c.h"
#include "lptim.h"
#include "types.h"

/*** SHT3x macros ***/

#define SHT3X_I2C_ADDRESS	0x44

/*** SHT3x structures ***/

/*!******************************************************************
 * \enum SHT3X_status_t
 * \brief SHT3X driver error codes.
 *******************************************************************/
typedef enum {
	SHT3X_SUCCESS = 0,
	SHT3X_ERROR_NULL_PARAMETER,
	SHT3X_ERROR_BASE_I2C = 0x0100,
	SHT3X_ERROR_BASE_LPTIM = (SHT3X_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	SHT3X_ERROR_BASE_LAST = (SHT3X_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} SHT3X_status_t;

/*** SHT3x functions ***/

/*!******************************************************************
 * \fn SHT3X_status_t SHT3X_perform_measurements(uint8_t i2c_address)
 * \brief Perform temperature and humidity measurements.
 * \param[in]  	i2c_address: I2C address of the sensor.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SHT3X_status_t SHT3X_perform_measurements(uint8_t i2c_address);

/*!******************************************************************
 * \fn SHT3X_status_t SHT3X_get_temperature(int8_t* temperature_degrees)
 * \brief Read temperature.
 * \param[in]  	none
 * \param[out] 	temperature_degrees: Pointer to signed byte that will contain the temperature in degrees.
 * \retval		Function execution status.
 *******************************************************************/
SHT3X_status_t SHT3X_get_temperature(int8_t* temperature_degrees);

/*!******************************************************************
 * \fn SHT3X_status_t SHT3X_get_humidity(uint8_t* humidity_percent)
 * \brief Read humidy.
 * \param[in]  	none
 * \param[out] 	humidity_percent: Pointer to byte that will contain the humidity in percent.
 * \retval		Function execution status.
 *******************************************************************/
SHT3X_status_t SHT3X_get_humidity(uint8_t* humidity_percent);

/*******************************************************************/
#define SHT3X_check_status(error_base) { if (sht3x_status != SHT3X_SUCCESS) { status = error_base + sht3x_status; goto errors; } }

/*******************************************************************/
#define SHT3X_stack_error(void) { ERROR_stack_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT3X); }

/*******************************************************************/
#define SHT3X_print_error(void) { ERROR_print_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT3X); }

#endif /* __SHT3X_H__ */
