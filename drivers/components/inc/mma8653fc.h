/*
 * mma8653fc.h
 *
 *  Created on: 1st may 2020
 *      Author: Ludo
 */

#ifndef __MMA8653FC_H__
#define __MMA8653FC_H__

#include "exti.h"
#include "i2c.h"
#include "math.h"
#include "mma8653fc_reg.h"
#include "types.h"

/*** MMA8653FC macros ***/

#define MMA8653FC_I2C_ADDRESS			0x1D
#define MMA8653FC_ACTIVE_CONFIG_LENGTH	10
#define MMA8653FC_SLEEP_CONFIG_LENGTH	3

/*** MMA8653FC structures ***/

/*!******************************************************************
 * \enum MMA8653FC_status_t
 * \brief MMA8653FC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	MMA8653FC_SUCCESS = 0,
	MMA8653FC_ERROR_NULL_PARAMETER,
	MMA8653FC_ERROR_CONFIG_SIZE,
	// Low level drivers errors.
	MMA8653FC_ERROR_BASE_I2C1 = 0x0100,
	MMA8653FC_ERROR_BASE_MATH = (MMA8653FC_ERROR_BASE_I2C1 + I2C_ERROR_BASE_LAST),
	// Last base value.
	MMA8653FC_ERROR_BASE_LAST = (MMA8653FC_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} MMA8653FC_status_t;

/*!******************************************************************
 * \enum MMA8653FC_register_setting_t
 * \brief MMA8653FC register structure.
 *******************************************************************/
typedef struct {
	uint8_t addr;
	uint8_t value;
} MMA8653FC_register_setting_t;

/*** MMA8653FC global variables ***/

extern const MMA8653FC_register_setting_t MMA8653FC_ACTIVE_CONFIG[MMA8653FC_ACTIVE_CONFIG_LENGTH];
extern const MMA8653FC_register_setting_t MMA8653FC_SLEEP_CONFIG[MMA8653FC_SLEEP_CONFIG_LENGTH];

/*** MMA8653FC functions ***/

/*!******************************************************************
 * \fn void MMA8653FC_init(EXTI_gpio_irq_cb_t motion_irq_callbac)
 * \brief Init MMA8653FC interface.
 * \param[in]  	motion_irq_callback: Motion interrupt callback.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MMA8653FC_init(EXTI_gpio_irq_cb_t motion_irq_callback);

/*!******************************************************************
 * \fn void MMA8653FC_de_init(void)
 * \brief Release MMA8653FC interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MMA8653FC_de_init(void);

/*!******************************************************************
 * \fn MMA8653FC_status_t MMA8653FC_get_id(uint8_t* chip_id)
 * \brief Read accelerometer chip ID.
 * \param[in]  	none
 * \param[out] 	chip_id: Pointer to the read chip ID.
 * \retval		Function execution status.
 *******************************************************************/
MMA8653FC_status_t MMA8653FC_get_id(uint8_t* chip_id);

/*!******************************************************************
 * \fn MMA8653FC_status_t MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, uint8_t mma8653fc_config_size)
 * \brief Set accelerometer configuration.
 * \param[in]  	mma8653fc_config: List of registers and values to set.
 * \param[in]	mma8653fc_config_size: Size of the configuration.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MMA8653FC_status_t MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, uint8_t mma8653fc_config_size);

/*!******************************************************************
 * \fn MMA8653FC_status_t MMA8653FC_get_data(int32_t* x, int32_t* y, int32_t* z)
 * \brief Read raw accelerometer data.
 * \param[in]  	none
 * \param[out] 	x: Pointer that will contain x axis acceleration value.
 * \param[out] 	y: Pointer that will contain y axis acceleration value.
 * \param[out] 	z: Pointer that will contain z axis acceleration value.
 * \retval		Function execution status.
 *******************************************************************/
MMA8653FC_status_t MMA8653FC_get_data(int32_t* x, int32_t* y, int32_t* z);

/*!******************************************************************
 * \fn void MMA8653_enable_motion_interrupt(void)
 * \brief Enable MMA8653FC motion interrupt.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MMA8653_enable_motion_interrupt(void);

/*!******************************************************************
 * \fn void MMA8653_disable_motion_interrupt(void)
 * \brief Disable MMA8653FC motion interrupt.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MMA8653_disable_motion_interrupt(void);

/*******************************************************************/
#define MMA8653FC_exit_error(base) { ERROR_check_exit(mma8653fc_status, MMA8653FC_SUCCESS, base) }

/*******************************************************************/
#define MMA8653FC_stack_error(base) { ERROR_check_stack(mma8653fc_status, MMA8653FC_SUCCESS, base) }

/*******************************************************************/
#define MMA8653FC_stack_exit_error(base, code) { ERROR_check_stack_exit(mma8653fc_status, MMA8653FC_SUCCESS, base, code) }

#endif /* __MMA8653FC_H__ */
