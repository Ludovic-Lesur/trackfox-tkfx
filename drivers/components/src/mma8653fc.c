/*
 * mma8653fc.c
 *
 *  Created on: 1st may 2020
 *      Author: Ludo
 */

#include "mma8653fc.h"

#include "error.h"
#include "exti.h"
#include "gpio.h"
#include "i2c.h"
#include "gpio_mapping.h"
#include "math.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "types.h"

/*** MMA8653FC local global variables ***/

const MMA8653FC_register_setting_t MMA8653FC_ACTIVE_CONFIG[MMA8653FC_ACTIVE_CONFIG_LENGTH] = {
	{MMA8653FC_REG_CTRL_REG1, 0x00}, // ACTIVE='0' (standby mode required to program registers).
	{MMA8653FC_REG_XYZ_DATA_CFG, 0x00}, // Full scale = +/-2g.
	{MMA8653FC_REG_CTRL_REG2, 0x1B}, // (S)MODS='11' (low power operation) and SLPE='0' (Auto sleep disabled).
	{MMA8653FC_REG_CTRL_REG3, 0x0A}, // WAKE_FF_MT='1' (motion interrupt wakes the sensor) and IPOL='1' (interrupt pin active high).
	{MMA8653FC_REG_FF_MT_CFG, 0x78}, // OAE='1' (motion detection). ELE='0' (latch disabled, bit automatically cleared). XEFE=YEFE=ZEFE='1' (any direction enabled).
	{MMA8653FC_REG_FF_MT_THS, 0x91}, // DBCNTM='1' and threshold value (1.071g).
	{MMA8653FC_REG_FF_MT_COUNT, 0x00}, // Debouncing counter not used.
	{MMA8653FC_REG_CTRL_REG5, 0x04}, // INT_CFG_FF_MT='1' (motion interrupt on INT1 pin).
	{MMA8653FC_REG_CTRL_REG4, 0x04}, // INT_EN_FF_MT='1' (motion interrupt enabled).
	{MMA8653FC_REG_CTRL_REG1, 0x39} // DR='111' (1Hz) and ACTIVE='1'.
};

const MMA8653FC_register_setting_t MMA8653FC_SLEEP_CONFIG[MMA8653FC_SLEEP_CONFIG_LENGTH] = {
	{MMA8653FC_REG_CTRL_REG2, 0x5B}, // RESET='1'.
	{MMA8653FC_REG_CTRL_REG2, 0x1B}, // RESET='0', (S)MODS='11' (low power operation) and SLPE='0' (auto sleep disabled).
	{MMA8653FC_REG_CTRL_REG3, 0x02}, // IPOL='1' (interrupt pin active high).
};

/*** MMA8653FC functions ***/

/*******************************************************************/
void MMA8653FC_init(EXTI_gpio_irq_cb_t motion_irq_callback) {
	// Configure interrupt pin.
	EXTI_configure_gpio(&GPIO_ACCELERO_IRQ, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, motion_irq_callback, NVIC_PRIORITY_ACCELEROMETER);
}

/*******************************************************************/
void MMA8653FC_de_init(void) {
	// Release interrupt pin.
	EXTI_release_gpio(&GPIO_ACCELERO_IRQ, GPIO_MODE_INPUT);
}

/*******************************************************************/
MMA8653FC_status_t MMA8653FC_get_id(uint8_t* chip_id) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	uint8_t local_addr = MMA8653FC_REG_WHO_AM_I;
	// Check parameter.
	if (chip_id == NULL) {
		status = MMA8653FC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read register.
	i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	i2c_status = I2C_read(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, chip_id, 1);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
errors:
	return status;
}

/*******************************************************************/
MMA8653FC_status_t MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, uint8_t mma8653fc_config_size) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
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
		i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, i2c_tx_data, 2, 1);
		I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	}
errors:
	return status;
}

/*******************************************************************/
MMA8653FC_status_t MMA8653FC_get_data(int32_t* x, int32_t* y, int32_t* z) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
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
	i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	i2c_status = I2C_read(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_X_LSB;
	i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	i2c_status = I2C_read(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	data |= (reg_data >> 6);
	math_status = MATH_two_complement_to_integer(data, 9, x);
	MATH_exit_error(MMA8653FC_ERROR_BASE_MATH);
	// Y-axis.
	local_addr = MMA8653FC_REG_OUT_Y_MSB;
	i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	i2c_status = I2C_read(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Y_LSB;
	i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	i2c_status = I2C_read(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	data |= (reg_data >> 6);
	math_status = MATH_two_complement_to_integer(data, 9, y);
	MATH_exit_error(MMA8653FC_ERROR_BASE_MATH);
	// Z-axis.
	local_addr = MMA8653FC_REG_OUT_Z_MSB;
	i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	i2c_status = I2C_read(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Z_LSB;
	i2c_status = I2C_write(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	i2c_status = I2C_read(I2C_INSTANCE_I2C1, MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C_exit_error(MMA8653FC_ERROR_BASE_I2C1);
	data |= (reg_data >> 6);
	math_status = MATH_two_complement_to_integer(data, 9, z);
	MATH_exit_error(MMA8653FC_ERROR_BASE_MATH);
errors:
	return status;
}

/*******************************************************************/
void MMA8653_enable_motion_interrupt(void) {
	// Enable interrupt.
	EXTI_enable_gpio_interrupt(&GPIO_ACCELERO_IRQ);
}

/*******************************************************************/
void MMA8653_disable_motion_interrupt(void) {
	// Enable interrupt.
    EXTI_disable_gpio_interrupt(&GPIO_ACCELERO_IRQ);
}
