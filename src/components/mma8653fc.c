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
#include "mode.h"

#if (defined SSM) || (defined ATM)

/*** MMA8653 local macros ***/

#define MMA8653FC_I2C_ADDRESS	0x1D

/*** MMA8653FC local global variables ***/

volatile unsigned char mma8653fc_motion_interrupt_flag = 0;

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
 * @param:	None.
 * @return:	None.
 */
unsigned char MMA8653FC_get_id(void) {
	unsigned char who_am_i = 0;
	unsigned char local_addr = MMA8653FC_REG_WHO_AM_I;
	unsigned char i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	i2c_access = I2C1_read(MMA8653FC_I2C_ADDRESS, &who_am_i, 1);
	return who_am_i;
}

/* CONFIGURE THE ACCELEROMETER.
 * @param mma8653fc_config:			Registers setting array.
 * @param mma8653fc_config_size:	Length of the config array.
 * @return:							None.
 */
void MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, unsigned char mma8653fc_config_size) {
	unsigned char i2c_access = 0;
	unsigned char i2c_tx_data[2];
	unsigned char reg_idx = 0;
	for (reg_idx=0 ; reg_idx<mma8653fc_config_size ; reg_idx++) {
		i2c_tx_data[0] = (mma8653fc_config[reg_idx].addr);
		i2c_tx_data[1] = (mma8653fc_config[reg_idx].value);
		i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, i2c_tx_data, 2, 1);
		if (i2c_access == 0) break;
	}
}

/* READ ACCELERATION DATA.
 * @param x:	Pointer to signed integer that will contain X-axis acceleration.
 * @param y:	Pointer to signed integer that will contain Y-axis acceleration.
 * @param z:	Pointer to signed integer that will contain Z-axis acceleration.
 * @return:		None.
 */
void MMA8653FC_get_data(signed int* x, signed int* y, signed int* z) {
	unsigned int data = 0;
	unsigned char reg_data = 0;
	unsigned char local_addr = 0;
	unsigned char i2c_access = 0;
	// X-axis.
	local_addr = MMA8653FC_REG_OUT_X_MSB;
	i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_X_LSB;
	i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data |= (reg_data >> 6);
	MATH_two_complement(data, 9, x);
	// Y-axis.
	local_addr = MMA8653FC_REG_OUT_Y_MSB;
	i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Y_LSB;
	i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data |= (reg_data >> 6);
	MATH_two_complement(data, 9, y);
	// Z-axis.
	local_addr = MMA8653FC_REG_OUT_Z_MSB;
	i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Z_LSB;
	i2c_access = I2C1_write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data |= (reg_data >> 6);
	MATH_two_complement(data, 9, z);
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
unsigned char MMA8653FC_get_motion_interrupt_flag(void) {
	return mma8653fc_motion_interrupt_flag;
}

#endif
