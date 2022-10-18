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
#include "types.h"

/*** MMA8653FC local global variables ***/

volatile uint8_t mma8653fc_motion_interrupt_flag = 0;

/*** MMA8653FC functions ***/

/* INIT MMA8653FC INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void MMA8653FC_init(void) {
	// Init flag.
	mma8653fc_motion_interrupt_flag = 0;
	// Configure interrupt pin.
	GPIO_configure(&GPIO_ACCELERO_IRQ, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_ACCELERO_IRQ, EXTI_TRIGGER_RISING_EDGE);
}

/* READ SENSOR ID.
 * @param chip_id:	Pointer that will contain chip ID.
 * @return status:	Function execution status.
 */
MMA8653FC_status_t MMA8653FC_get_id(uint8_t* chip_id) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint8_t local_addr = MMA8653FC_REG_WHO_AM_I;
	// Read register.
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, chip_id, 1);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
errors:
	return status;
}

/* CONFIGURE THE ACCELEROMETER.
 * @param mma8653fc_config:			Registers setting array.
 * @param mma8653fc_config_size:	Length of the config array.
 * @return status:					Function execution status.
 */
MMA8653FC_status_t MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, uint8_t mma8653fc_config_size) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint8_t i2c_tx_data[2];
	uint8_t reg_idx = 0;
	// Write configuration.
	for (reg_idx=0 ; reg_idx<mma8653fc_config_size ; reg_idx++) {
		i2c_tx_data[0] = (mma8653fc_config[reg_idx].addr);
		i2c_tx_data[1] = (mma8653fc_config[reg_idx].value);
		i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, i2c_tx_data, 2, 1);
		I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	}
errors:
	return status;
}

/* READ ACCELERATION DATA.
 * @param x:		Pointer to int32_teger that will contain X-axis acceleration.
 * @param y:		Pointer to int32_teger that will contain Y-axis acceleration.
 * @param z:		Pointer to int32_teger that will contain Z-axis acceleration.
 * @return status:	Function execution status.
 */
MMA8653FC_status_t MMA8653FC_get_data(int32_t* x, int32_t* y, int32_t* z) {
	// Local variables.
	MMA8653FC_status_t status = MMA8653FC_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint32_t data = 0;
	uint8_t reg_data = 0;
	uint8_t local_addr = 0;
	// X-axis.
	local_addr = MMA8653FC_REG_OUT_X_MSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_X_LSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	data |= (reg_data >> 6);
	MATH_two_complement(data, 9, x);
	// Y-axis.
	local_addr = MMA8653FC_REG_OUT_Y_MSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Y_LSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	data |= (reg_data >> 6);
	MATH_two_complement(data, 9, y);
	// Z-axis.
	local_addr = MMA8653FC_REG_OUT_Z_MSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Z_LSB;
	i2c1_status = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	I2C1_status_check(MMA8653FC_ERROR_BASE_I2C);
	data |= (reg_data >> 6);
	MATH_two_complement(data, 9, z);
errors:
	return status;
}


/* SET MOTION INTERRUPT STATUS.
 * @param:	None.
 * @return:	None.
 */
void MMA8653FC_set_motion_interrupt_flag(void) {
	mma8653fc_motion_interrupt_flag = 1;
}

/* CLEAR MOTION INTERRUPT STATUS.
 * @param:	None.
 * @return:	None.
 */
void MMA8653FC_clear_motion_interrupt_flag(void) {
	mma8653fc_motion_interrupt_flag = 0;
}

/* GET MOTION INTERRUPT STATUS.
 * @param:									None.
 * @return mma8653fc_motion_interrupt_flag:	'1' if the motion interrupt occurred (EXTI line), '0' otherwise.
 */
uint8_t MMA8653FC_get_motion_interrupt_flag(void) {
	return mma8653fc_motion_interrupt_flag;
}
