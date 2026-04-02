/*
 * accelerometer.h
 *
 *  Created on: 16 nov. 2020
 *      Author: Ludo
 */

#ifndef __ACCELEROMETER_H__
#define __ACCELEROMETER_H__

#include "fxls89xxxx.h"
#include "i2c_address.h"
#include "mma865xfc.h"

/*** ACCELEROMETER macros ***/

#ifdef HW2_0

#define ACCELEROMETER_CONFIGURATION_SIZE_ACTIVE     12
#define ACCELEROMETER_CONFIGURATION_SIZE_SLEEP      1

#define ACCELEROMETER_I2C_ADDRESS                   I2C_ADDRESS_FXLS8974CF

#define accelerometer_status                        fxls89xxxx_status
#define ACCELEROMETER_SUCCESS                       FXLS89XXXX_SUCCESS
#define ACCELEROMETER_init                          FXLS89XXXX_init
#define ACCELEROMETER_de_init                       FXLS89XXXX_de_init
#define ACCELEROMETER_write_configuration           FXLS89XXXX_write_configuration
#define ACCELEROMETER_stack_error(void)             FXLS89XXXX_stack_error(ERROR_BASE_FXLS8974CF)

#define ACCELEROMETER_ERROR_BASE                    ERROR_BASE_FXLS8974CF
#define ACCELEROMETER_POWER_DRIVER_ERROR            POWER_ERROR_DRIVER_FXLS89XXXX

#else

#define ACCELEROMETER_CONFIGURATION_SIZE_ACTIVE     10
#define ACCELEROMETER_CONFIGURATION_SIZE_SLEEP      3

#define ACCELEROMETER_I2C_ADDRESS                   I2C_ADDRESS_MMA8653FC

#define accelerometer_status                        mma865xfc_status
#define ACCELEROMETER_SUCCESS                       MMA865XFC_SUCCESS
#define ACCELEROMETER_init                          MMA865XFC_init
#define ACCELEROMETER_de_init                       MMA865XFC_de_init
#define ACCELEROMETER_write_configuration           MMA865XFC_write_configuration
#define ACCELEROMETER_stack_error(void)             MMA865XFC_stack_error(ERROR_BASE_MMA8653FC)

#define ACCELEROMETER_ERROR_BASE                    ERROR_BASE_MMA8653FC
#define ACCELEROMETER_POWER_DRIVER_ERROR            POWER_ERROR_DRIVER_MMA865XFC

#endif /* HW2.0 */

/*** ACCELEROMETER structures ***/

#ifdef HW2_0
typedef FXLS89XXXX_status_t ACCELEROMETER_status_t;
typedef FXLS89XXXX_register_setting_t ACCELEROMETER_register_setting_t;
#else
typedef MMA865XFC_status_t ACCELEROMETER_status_t;
typedef MMA865XFC_register_setting_t ACCELEROMETER_register_setting_t;
#endif

/*** ACCELEROMETER global variables ***/

extern const ACCELEROMETER_register_setting_t ACCELEROMETER_CONFIGURATION_ACTIVE[ACCELEROMETER_CONFIGURATION_SIZE_ACTIVE];
extern const ACCELEROMETER_register_setting_t ACCELEROMETER_CONFIGURATION_SLEEP[ACCELEROMETER_CONFIGURATION_SIZE_SLEEP];

#endif /* __ACCELEROMETER_H__ */
