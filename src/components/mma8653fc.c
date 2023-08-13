/*
 * mma8653fc.c
 *
 *  Created on: 1st may 2020
 *      Author: Ludo
 */

#include "mma8653fc.h"

#include "exti.h"
#include "gpio.h"
#include "i2c.h"
#include "mapping.h"
#include "math.h"
#include "nvic.h"
#include "types.h"

/*** MMA8653FC local global variables ***/

volatile uint8_t mma8653fc_motion_interrupt_flag = 0;

/*******************************************************************/
static void _MMA8653FC_set_motion_interrupt_flag(void) {
	mma8653fc_motion_interrupt_flag = 1;
}

/*** MMA8653FC functions ***/

/*******************************************************************/
void MMA8653FC_init(void) {
	// Init flag.
	mma8653fc_motion_interrupt_flag = 0;
	// Configure interrupt pin.
	GPIO_configure(&GPIO_ACCELERO_IRQ, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_ACCELERO_IRQ, EXTI_TRIGGER_RISING_EDGE, &_MMA8653FC_set_motion_interrupt_flag);
}

/*******************************************************************/
void MMA8653FC_de_init(void) {
	// Release interrupt pin.
	EXTI_release_gpio(&GPIO_ACCELERO_IRQ);
}

/*******************************************************************/
MMA8653FC_status_t MMA8653FC_get_id(uint8_t* chip_id) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint8_t local_addr = MMA8653FC_REG_WHO_AM_I;
	// Check parameter.
	if (chip_id == NULL) {
		status = MMA8653FC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read register.
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, chip_id, 1);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
MMA8653FC_status_t MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, uint8_t mma8653fc_config_size) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint8_t i2c_tx_data[2];
	uint8_t reg_idx = 0;
	if (mma8653fc_config == NULL) {
		status = MMA8653FC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (mma8653fc_config_size == 0) {
		status = MMA8653FC_ERROR_CONFIG_SIZE;
		goto errors;
	}
	// Write configuration.
	for (reg_idx=0 ; reg_idx<mma8653fc_config_size ; reg_idx++) {
		i2c_tx_data[0] = (mma8653fc_config[reg_idx].addr);
		i2c_tx_data[1] = (mma8653fc_config[reg_idx].value);
		i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, i2c_tx_data, 2, 1);
		I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	}
errors:
	return status;
}

/*******************************************************************/
MMA8653FC_status_t MMA8653FC_get_data(int32_t* x, int32_t* y, int32_t* z) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	uint32_t data = 0;
	uint8_t reg_data = 0;
	uint8_t local_addr = 0;
	// Check parameters.
	if ((x == NULL) || (y == NULL) || (z == NULL)) {
		status = MMA8653FC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// X-axis.
	local_addr = MMA8653FC_REG_OUT_X_MSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_X_LSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	data |= (reg_data >> 6);
	math_status = MATH_two_complement_to_int32(data, 9, x);
	MATH_check_status(MMA8653FC_ERROR_BASE_MATH);
	// Y-axis.
	local_addr = MMA8653FC_REG_OUT_Y_MSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Y_LSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	data |= (reg_data >> 6);
	math_status = MATH_two_complement_to_int32(data, 9, y);
	MATH_check_status(MMA8653FC_ERROR_BASE_MATH);
	// Z-axis.
	local_addr = MMA8653FC_REG_OUT_Z_MSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Z_LSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_check_status(MMA8653FC_ERROR_BASE_I2C);
	data |= (reg_data >> 6);
	math_status = MATH_two_complement_to_int32(data, 9, z);
	MATH_check_status(MMA8653FC_ERROR_BASE_MATH);
errors:
	return status;
}

/*******************************************************************/
void MMA8653_enable_motion_interrupt(void) {
	// Enable interrupt.
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI_0_1, NVIC_PRIORITY_EXTI_0_1);
}

/*******************************************************************/
void MMA8653_disable_motion_interrupt(void) {
	// Enable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_EXTI_0_1);
}

/*******************************************************************/
volatile uint8_t MMA8653FC_get_motion_interrupt_flag(void) {
	return mma8653fc_motion_interrupt_flag;
}

/*******************************************************************/
void MMA8653FC_clear_motion_interrupt_flag(void) {
	mma8653fc_motion_interrupt_flag = 0;
}
